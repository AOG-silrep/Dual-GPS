#!/usr/bin/env python3
"""Load a u-center config file (.txt) into a ZED-F9P, over USB or a UART.

The file is what u-center's Receiver Configuration tool saves for a Gen 9
receiver: a MON-VER line, then CFG-VALGET dumps in hex.

Only the keys whose RAM value differs from the file are written, to RAM and
Flash. Then the receiver is reset, and its RAM, which after a boot holds the
defaults plus Flash plus BBR, is checked against the file. Learned on the bench
F9P (HPG 1.13): reading the Flash layer can reset it, and rewriting a whole
1212-key file to Flash reset it partway through, so neither is done here.
Running it again is safe: it only writes what still differs.

  ./upload_f9p_config.py "F9P 1.13 config single serial tile plow GPGGA only.txt"
  ./upload_f9p_config.py -n CONFIG.txt            show what would change
  ./upload_f9p_config.py -p ttyUSB1 -b 115200 CONFIG.txt
"""
import argparse
import collections
import glob
import os
import struct
import subprocess
import sys
import time

import serial

SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}  # value bytes, by the key's size bits
LAYER_RAM, LAYER_BBR = 0, 1  # CFG-VALGET layer
SET_RAM, SET_BBR, SET_FLASH = 1, 2, 4  # CFG-VALSET layer bits
GROUPS = {
    0x03: "CFG-TMODE", 0x05: "CFG-TP", 0x09: "CFG-RTCM", 0x11: "CFG-NAVSPG", 0x14: "CFG-NAVHPG",
    0x21: "CFG-RATE", 0x22: "CFG-ODO", 0x24: "CFG-GEOFENCE", 0x25: "CFG-MOT", 0x31: "CFG-SIGNAL",
    0x34: "CFG-BDS", 0x35: "CFG-GAL", 0x36: "CFG-SBAS", 0x37: "CFG-QZSS", 0x41: "CFG-ITFM",
    0x51: "CFG-I2C", 0x52: "CFG-UART1", 0x53: "CFG-UART2", 0x64: "CFG-SPI", 0x65: "CFG-USB",
    0x71: "CFG-I2CINPROT", 0x72: "CFG-I2COUTPROT", 0x73: "CFG-UART1INPROT", 0x74: "CFG-UART1OUTPROT",
    0x75: "CFG-UART2INPROT", 0x76: "CFG-UART2OUTPROT", 0x77: "CFG-USBINPROT", 0x78: "CFG-USBOUTPROT",
    0x79: "CFG-SPIINPROT", 0x7A: "CFG-SPIOUTPROT", 0x91: "CFG-MSGOUT", 0x92: "CFG-INFMSG",
    0x93: "CFG-NMEA", 0xA2: "CFG-TXREADY", 0xA3: "CFG-HW", 0xC7: "CFG-RINV", 0xD0: "CFG-PM",
    0xDE: "CFG-LOGFILTER", 0xF6: "CFG-SEC",
}
# the settings of the port we talk over are written last; by CFG-PRT portID
OWN_PORT_GROUPS = {1: (0x52, 0x73, 0x74), 2: (0x53, 0x75, 0x76), 3: (0x65, 0x77, 0x78)}
UART_BAUD_KEY = {1: 0x40520001, 2: 0x40530001}


def die(msg):
    print("ERROR: " + msg, file=sys.stderr)
    sys.exit(1)


def stage(n, text):
    print(f"[{n}/5] {text}", flush=True)


def ck(b):
    a = c = 0
    for x in b:
        a = (a + x) & 0xFF
        c = (c + a) & 0xFF
    return bytes([a, c])


def frame(cls, mid, pl=b""):
    body = bytes([cls, mid]) + struct.pack("<H", len(pl)) + pl
    return b"\xb5\x62" + body + ck(body)


def decode_kv(data):
    out, i = [], 0
    while i < len(data):
        key = struct.unpack_from("<I", data, i)[0]
        n = SIZE.get(key >> 28 & 7)
        if n is None:
            raise ValueError(f"key {key:08X} has no valid size")
        out.append((key, data[i + 4:i + 4 + n]))
        i += 4 + n
    return out


def mon_ver(pl):
    fields = [pl[0:30], pl[30:40]] + [pl[i:i + 30] for i in range(40, len(pl), 30)]
    return [f.split(b"\0")[0].decode("latin-1") for f in fields]


def fwver(strings):
    return next((s[6:] for s in strings or [] if s.startswith("FWVER=")), "unknown firmware")


def parse_file(path):
    """-> (MON-VER strings or None, {key: value bytes}) from a u-center config .txt"""
    fw, kv = None, {}
    try:
        lines = open(path, encoding="latin-1").read().splitlines()
    except OSError as e:
        die(f"can't read {path}: {e}")
    for line in lines:
        if " - " not in line:
            continue
        name, hexs = (x.strip() for x in line.split(" - ", 1))
        try:
            b = bytes.fromhex(hexs)
            n = b[2] | b[3] << 8
            pl = b[4:4 + n]
            if len(pl) != n:
                raise ValueError("short line")
            if name == "MON-VER":
                fw = mon_ver(pl)
            elif name == "CFG-VALGET":
                kv.update(decode_kv(pl[4:]))
        except (ValueError, IndexError) as e:
            die(f"{path}: bad {name} line ({e}); is it a u-center Gen 9 config file?")
    if not kv:
        die(f"{path} holds no CFG-VALGET lines; is it a u-center Gen 9 config file?")
    return fw, kv


def key_name(key):
    g = key >> 16 & 0xFF
    return f"{GROUPS.get(g, f'group {g:02X}')} {key:08X}"


def value(v):
    if v is None:
        return "absent"
    return str(int.from_bytes(v, "little")) if len(v) <= 4 else v.hex()


class Ubx:
    def __init__(self, port, baud):
        self.s = serial.Serial(port, baud, timeout=0.05, exclusive=True)
        self.buf = b""

    def close(self):
        self.s.close()

    def send(self, cls, mid, pl=b""):
        self.s.reset_input_buffer()
        self.buf = b""
        self.s.write(frame(cls, mid, pl))

    def read_msg(self, deadline):
        while time.time() < deadline:
            self.buf += self.s.read(4096)
            while True:
                i = self.buf.find(b"\xb5\x62")
                if i < 0:
                    self.buf = self.buf[-1:]
                    break
                self.buf = self.buf[i:]
                if len(self.buf) < 6:
                    break
                n = self.buf[4] | self.buf[5] << 8
                if len(self.buf) < 8 + n:
                    break
                m, self.buf = self.buf[:8 + n], self.buf[8 + n:]
                if ck(m[2:6 + n]) == m[6 + n:]:
                    return m[2], m[3], m[6:6 + n]
        return None

    def poll(self, cls, mid, pl=b"", timeout=2.0):
        """-> reply payload, or None on a NAK"""
        self.send(cls, mid, pl)
        end = time.time() + timeout
        while r := self.read_msg(end):
            if r[0] == cls and r[1] == mid:
                return r[2]
            if r[0] == 0x05 and r[1] == 0x00 and r[2][:2] == bytes([cls, mid]):
                return None
        raise TimeoutError(f"no reply to UBX {cls:02X}-{mid:02X}")

    def acked(self, cls, mid, pl, timeout=3.0):
        self.send(cls, mid, pl)
        end = time.time() + timeout
        while r := self.read_msg(end):
            if r[0] == 0x05 and r[2][:2] == bytes([cls, mid]):
                return r[1] == 0x01
        raise TimeoutError(f"no ACK for UBX {cls:02X}-{mid:02X}")

    def valget(self, layer, keys):
        """-> {key: value bytes}; keys the receiver doesn't know are left out"""
        got = {}
        for c in range(0, len(keys), 64):
            chunk = keys[c:c + 64]
            r = self.poll(0x06, 0x8B, struct.pack("<BBH", 0, layer, 0) + b"".join(struct.pack("<I", k) for k in chunk))
            if r is not None:
                got.update(decode_kv(r[4:]))
                continue
            for k in chunk:  # one unknown key NAKs the lot
                r = self.poll(0x06, 0x8B, struct.pack("<BBHI", 0, layer, 0, k))
                if r is not None:
                    got.update(decode_kv(r[4:]))
        return got

    def valset(self, layers, items):
        """CFG-VALSET in 64-key messages -> keys the receiver rejected"""
        rejected = []
        for c in range(0, len(items), 64):
            chunk = items[c:c + 64]
            kv = b"".join(struct.pack("<I", k) + v for k, v in chunk)
            if self.acked(0x06, 0x8A, struct.pack("<BBH", 0, layers, 0) + kv):
                print(f"  {len(chunk)} keys: ACK", flush=True)
                continue
            print(f"  {len(chunk)} keys: NAK, trying them one at a time", flush=True)
            for k, v in chunk:
                if not self.acked(0x06, 0x8A, struct.pack("<BBHI", 0, layers, 0, k) + v):
                    rejected.append(k)
        return rejected


def vendor(path):
    r = subprocess.run(["udevadm", "info", "-q", "property", "-n", path], capture_output=True, text=True)
    return next((ln.split("=", 1)[1] for ln in r.stdout.splitlines() if ln.startswith("ID_VENDOR_ID=")), "")


def find_port(port, uart):
    """-> (stable path to open, device node, True if it's the receiver's own USB)"""
    links = sorted(glob.glob("/dev/serial/by-path/*-usb-*"))
    if port:
        if "/" not in port and os.path.exists("/dev/" + port):
            port = "/dev/" + port  # accept "ttyACM0"
        if not os.path.exists(port):
            die(f"no such port: {port} (see ls -l /dev/serial/by-id)")
    else:
        if uart:
            die("-b (UART) needs -p PORT, the USB-serial adapter wired to the F9P")
        # by-id can't tell two F9Ps apart (no serial number), so look per USB socket
        found = [ln for ln in links if vendor(ln) == "1546"]
        if not found:
            die("no u-blox USB receiver found: plug in the F9P's USB, or give -p PORT")
        if len(found) > 1:
            die(f"{len(found)} u-blox receivers are plugged in, pick one with -p:\n  " + "\n  ".join(found))
        port = found[0]
    dev = os.path.realpath(port)
    # the receiver comes back as a new ttyACM after a reset; its by-path name stays the same
    link = next((ln for ln in links if os.path.realpath(ln) == dev), port)
    return link, dev, not uart and vendor(dev) == "1546"


def reset(u, link, baud, usb):
    """reset the receiver and reopen it -> Ubx"""
    u.s.write(frame(0x06, 0x04, struct.pack("<HBB", 0x0000, 0x01, 0)))  # CFG-RST: hot start, software reset
    u.s.flush()
    time.sleep(0.2)
    u.close()
    t = time.time()
    if usb:  # it drops off USB first
        while os.path.exists(link) and time.time() - t < 5:
            time.sleep(0.05)
    else:
        time.sleep(1)
    end = time.time() + 20
    while time.time() < end:
        if os.path.exists(link):
            time.sleep(0.5)
            try:
                nu = Ubx(link, baud)
            except (serial.SerialException, OSError):
                continue
            try:
                st = nu.poll(0x01, 0x03, timeout=1.5)  # NAV-STATUS
                up = struct.unpack_from("<I", st, 12)[0] / 1000 if st else None
                print(f"  reset; back on {os.path.realpath(link)}" + (f", up {up:.1f} s" if up is not None else ""))
                if up is not None and up > 60:
                    print("  WARNING: that uptime is long; it may not have reset")
                return nu
            except (serial.SerialException, OSError, TimeoutError):
                nu.close()
        time.sleep(0.3)
    die(f"the receiver didn't come back on {link} within 20 s of the reset")


def write(u, want, todo, layers, port_id, baud):
    """VALSET todo, the settings of the port we're on last -> (rejected keys, baud now)"""
    own = OWN_PORT_GROUPS.get(port_id, ())
    baud_key = UART_BAUD_KEY.get(port_id)
    last = [k for k in todo if (k >> 16 & 0xFF) in own and k != baud_key]
    first = [k for k in todo if k not in last and k != baud_key]
    rejected = []
    try:
        for group in (first, last):
            rejected += u.valset(layers, [(k, want[k]) for k in group])
        if baud_key in todo:  # our own UART's baud: the ACK may come at either rate
            u.send(0x06, 0x8A, struct.pack("<BBHI", 0, layers, 0, baud_key) + want[baud_key])
            time.sleep(0.3)
            baud = int.from_bytes(want[baud_key], "little")
            u.s.baudrate = baud
            print(f"  this UART's baud -> {baud}")
    except TimeoutError as e:
        die(f"{e}: the receiver may have reset. Run this again; it only writes what still differs.")
    return rejected, baud


def check(u, want, keys):
    """-> keys whose RAM value differs from the file"""
    got = u.valget(LAYER_RAM, keys)
    return [k for k in keys if k in got and got[k] != want[k]], got


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], epilog=__doc__.split("\n\n", 1)[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="u-center config file (.txt)")
    ap.add_argument("-p", "--port", help="receiver port, e.g. ttyACM0 (default: the one u-blox USB receiver)")
    ap.add_argument("-b", "--baud", type=int, help="talk over a UART at this baud (USB-serial adapter)")
    ap.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation")
    ap.add_argument("-n", "--dry-run", action="store_true", help="show what would change and stop")
    ap.add_argument("-v", "--verbose", action="store_true", help="list every key that changes")
    ap.add_argument("--no-reset", action="store_true", help="don't reset the receiver to check the saved config")
    a = ap.parse_args()

    stage(1, "Reading config file")
    fw, want = parse_file(a.config)
    keys = list(want)
    print(f"  {os.path.basename(a.config)}: {len(keys)} keys, saved from {fwver(fw)}")

    stage(2, "Finding receiver")
    link, dev, usb = find_port(a.port, a.baud)
    print(f"  port: {link} -> {dev}")
    if subprocess.run(["fuser", "-s", dev], stderr=subprocess.DEVNULL).returncode == 0:
        die(f"{dev} is open in another program (u-center, a serial monitor, field_test.py?): close it first")
    baud = a.baud or 115200  # ignored on USB
    try:
        u = Ubx(link, baud)
        ver = mon_ver(u.poll(0x0A, 0x04) or b"")
    except (serial.SerialException, OSError, TimeoutError) as e:
        die(f"the receiver doesn't answer on {link} ({e}). Check -b, or power-cycle it.")
    print(f"  receiver: {' | '.join(ver)}")
    if ver[0].startswith("ROM BOOT"):
        die("it's in safeboot (ROM loader), e.g. after an interrupted firmware update: finish that first")
    if fw and fwver(fw) != fwver(ver):
        print(f"  WARNING: the file was saved from {fwver(fw)} but the receiver runs {fwver(ver)}")
    try:
        prt = u.poll(0x06, 0x00)  # CFG-PRT of the port we're on
        port_id = prt[0] if prt else None
    except TimeoutError:
        port_id = None

    stage(3, "Comparing with the receiver")
    ram = u.valget(LAYER_RAM, keys)
    unknown = [k for k in keys if k not in ram]
    known = [k for k in keys if k in ram]
    todo = [k for k in known if ram[k] != want[k]]
    print(f"  {len(todo)} of {len(keys)} keys differ from the receiver's RAM")
    if unknown:
        print(f"  {len(unknown)} keys the receiver doesn't know, skipped: " + " ".join(f"{k:08X}" for k in unknown))
    if not todo:
        print("  The receiver's RAM already matches the file. To check it's saved too, power-cycle it and run this again.")
        return 0
    by_group = collections.Counter(key_name(k).rsplit(" ", 1)[0] for k in todo)
    print("  by group: " + ", ".join(f"{g} {n}" for g, n in by_group.most_common()))
    shown = todo if a.verbose else todo[:30]
    for k in shown:
        print(f"    {key_name(k)}: {value(ram[k])} -> {value(want[k])}")
    if len(shown) < len(todo):
        print(f"    ... and {len(todo) - len(shown)} more (-v lists them all)")
    if a.dry_run:
        print("Dry run, nothing written.")
        return 0
    if not a.yes:
        try:
            ans = input(f"Write {len(todo)} keys to the receiver's RAM and Flash? [y/N] ")
        except EOFError:
            ans = ""
        if not ans.lower().startswith("y"):
            print("Cancelled.")
            return 1

    stage(4, f"Writing {len(todo)} keys to RAM and Flash")
    rejected, baud = write(u, want, todo, SET_RAM | SET_FLASH, port_id, baud)

    if a.no_reset:
        stage(5, "Checking RAM (not reset, so the saved copy isn't checked)")
        bad, got = check(u, want, known)
    else:
        stage(5, "Resetting the receiver and checking what it boots with")
        u = reset(u, link, baud, usb)
        bad, got = check(u, want, known)
        # BBR overrides Flash at boot: keys still wrong and held there get written there too
        held = u.valget(LAYER_BBR, [k for k in bad if k not in rejected])
        if held:
            print(f"  {len(held)} keys are held in BBR, which overrides Flash at boot: writing them there too")
            more, baud = write(u, want, list(held), SET_RAM | SET_BBR | SET_FLASH, port_id, baud)
            rejected += more
            u = reset(u, link, baud, usb)
            bad, got = check(u, want, known)
    bad = [k for k in bad if k not in rejected]
    if rejected:
        print(f"  the receiver rejected {len(rejected)} keys: " + " ".join(f"{k:08X}" for k in rejected))
    if bad:
        print(f"  {len(bad)} keys don't match the file:")
        for k in bad[:40]:
            print(f"    {key_name(k)}: {value(got.get(k))}, file {value(want[k])}")
        return 1
    print(f"  All {len(known)} keys match the file" + ("" if a.no_reset else " after the reboot") + ".")
    return 0


if __name__ == "__main__":
    sys.exit(main())
