#!/usr/bin/env bash
# Upload a u-blox firmware image (.bin) to a ZED-F9P from Linux.
#
# u-blox only ships the updater for Windows, so this runs u-center's
# ubxfwupdate.exe under wine the way u-center itself does: with -p STDIO, while
# a small relay (python + pyserial) passes the bytes to the receiver, follows
# its <AC>Setbaud/<AC>Reenum requests and reopens the port when the receiver
# drops off USB and comes back.
#
# The update goes through safeboot, on USB too: the "USB alternative" method
# needs the running firmware to invalidate itself, which HPG 1.13 refuses. If
# the receiver is already in safeboot (an interrupted update), the script sees
# "ROM BOOT" in its version and carries on from there.

set -euo pipefail

usage() {
	cat <<'EOF'
Usage: upload_f9p_firmware.sh [-p PORT] [-b BAUD] [-y] [-n] FIRMWARE.bin [-- extra ubxfwupdate args]

  -p PORT  receiver port, e.g. ttyACM0 (default: the one u-blox USB receiver plugged in)
  -b BAUD  update over UART1 (3.3 V USB-serial adapter) whose current baud is BAUD
  -y       don't ask for confirmation
  -n       dry run: print the wine command and stop

Examples:
  ./upload_f9p_firmware.sh UBX_F9_100_HPG_132_ZED_F9P.<md5>.bin
  ./upload_f9p_firmware.sh -p /dev/ttyUSB1 -b 115200 UBX_F9_100_HPG_132_ZED_F9P.<md5>.bin
  ./upload_f9p_firmware.sh FW.bin -- --flashret 1    keep high-value flash data

Environment: WINEPREFIX (default ~/.wine), UBXFWUPDATE (path to ubxfwupdate.exe)
Get the firmware from the ZED-F9P page on u-blox.com (Documentation & resources).
The update erases the whole flash, config included: reload the config file afterwards.
Over USB it takes about a minute. If it gets interrupted, run it again: it resumes.
EOF
}

die() { echo "ERROR: $*" >&2; exit 1; }

# training sequence then MON-VER poll, as ubxfwupdate starts (the safeboot ROM
# needs the training first); prints the version strings, or nothing
probe() { # PORT BAUD
	python3 -I - "$1" "$2" 2>/dev/null <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=0.1, exclusive=True)
for _ in range(2):
    s.write(b"\x55\x55")
    s.flush()
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write(bytes.fromhex("b5620a0400000e34"))
    buf, end = b"", time.time() + 1.5
    while time.time() < end:
        buf += s.read(4096)
        i = buf.find(b"\xb5\x62\x0a\x04")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 8 + (n := buf[i + 4] | buf[i + 5] << 8):
            pl = buf[i + 6:i + 6 + n]
            f = [pl[0:30], pl[30:40]] + [pl[j:j + 30] for j in range(40, n, 30)]
            print(" | ".join(x.split(b"\0")[0].decode("latin-1") for x in f))
            sys.exit()
PY
}
warn() { echo "  WARNING: $*" >&2; }
stage() { echo "[$1/4] $2"; }
mmss() { printf '%d:%02d' $(($1 / 60)) $(($1 % 60)); }

[[ ${1:-} == --help ]] && { usage; exit 0; }
port= baud= yes=0 dry=0
while getopts ':p:b:ynh' o; do
	case $o in
		p) port=$OPTARG ;;
		b) baud=$OPTARG ;;
		y) yes=1 ;;
		n) dry=1 ;;
		h) usage; exit 0 ;;
		*) usage >&2; exit 1 ;;
	esac
done
shift $((OPTIND - 1))
(($# >= 1)) || { usage >&2; exit 1; }
bin=$1
shift
[[ ${1:-} == -- ]] && shift
extra=("$@")
[[ -z $baud || $baud =~ ^[0-9]+$ ]] || die "-b needs a baud rate, got '$baud'"

stage 1 "Checking tools and image"
command -v wine >/dev/null || die "wine is not installed (sudo apt install wine)"
python3 -I -c 'import serial' 2>/dev/null || die "needs python3 with pyserial (sudo apt install python3-serial)"
export WINEPREFIX=${WINEPREFIX:-$HOME/.wine}
if [[ -z ${UBXFWUPDATE:-} ]]; then
	shopt -s nullglob
	exes=("$WINEPREFIX"/drive_c/Program\ Files*/u-blox/u-center*/ubxfwupdate.exe)
	shopt -u nullglob
	((${#exes[@]})) || die "ubxfwupdate.exe not found: install u-center (Windows) under wine, or set UBXFWUPDATE"
	UBXFWUPDATE=$(printf '%s\n' "${exes[@]}" | sort -V | tail -n 1)
fi
[[ -f $UBXFWUPDATE ]] || die "no such file: $UBXFWUPDATE"
echo "  tool: $UBXFWUPDATE"

[[ -f $bin ]] || die "no such file: $bin"
bin=$(readlink -f -- "$bin")
name=${bin##*/}
shopt -s nocasematch
[[ $name == *ZED[_-]F9P* ]] || die "$name is not a ZED-F9P image (its name has no ZED_F9P)"
shopt -u nocasematch
echo "  image: $name ($(stat -c %s -- "$bin") bytes), ZED-F9P image"
if [[ $name =~ ([0-9a-fA-F]{32}) ]]; then
	want=${BASH_REMATCH[1],,}
	got=$(md5sum -- "$bin" | cut -d ' ' -f 1)
	[[ $got == "$want" ]] || die "the file's MD5 is $got but its name says $want: corrupt or partial download? Download it again."
	echo "  MD5 ok (matches the file name)"
else
	echo "  no MD5 in the file name, not checked"
fi

stage 2 "Finding receiver"
if [[ -z $port ]]; then
	[[ -z $baud ]] || die "-b (UART) needs -p PORT, the USB-serial adapter wired to the F9P"
	# by-id can't tell two F9Ps apart (no serial number), so look per USB socket
	found=()
	for l in /dev/serial/by-path/*-usb-*; do
		[[ -e $l ]] || continue
		[[ $(udevadm info -q property -n "$l" 2>/dev/null) == *ID_VENDOR_ID=1546* ]] && found+=("$l")
	done
	case ${#found[@]} in
		0) die "no u-blox USB receiver found: plug in the F9P's USB, or give -p PORT" ;;
		1) port=${found[0]} ;;
		*) die "${#found[@]} u-blox receivers are plugged in, pick one with -p:$(printf '\n  %s' "${found[@]}")" ;;
	esac
fi
[[ $port == */* || ! -e /dev/$port ]] || port=/dev/$port # accept "ttyACM0"
[[ -e $port ]] || die "no such port: $port (see ls -l /dev/serial/by-id)"
dev=$(readlink -f -- "$port")
[[ -c $dev ]] || die "$port is not a serial device"
[[ -r $dev && -w $dev ]] || die "no permission for $dev (add yourself to the dialout group)"
# the receiver comes back as a new ttyACM after a reset; its by-path name stays the same
link=$port
for l in /dev/serial/by-path/*-usb-*; do
	[[ -e $l && $(readlink -f -- "$l") == "$dev" ]] && { link=$l; break; }
done
echo "  port: $link -> $dev"
if fuser -s "$dev" 2>/dev/null; then
	die "$dev is open in another program (u-center, a serial monitor, field_test.py?): close it first"
fi
echo "  port free"
ver=$(probe "$dev" "${baud:-115200}")
if [[ -z $ver && -n $baud && $baud != 9600 ]]; then
	ver=$(probe "$dev" 9600) # the safeboot ROM talks at 9600 on the UART
	[[ $ver == "ROM BOOT"* ]] && baud=9600
fi
[[ -n $ver ]] || die "the receiver doesn't answer a version poll on $dev. Check -b, or power-cycle it (unplug and replug) and retry."
echo "  receiver: $ver"
if [[ $ver == "ROM BOOT"* ]]; then
	safeboot=0
	echo "  it's already in safeboot (ROM loader), e.g. after an interrupted update: carrying on from there"
else
	safeboot=1
fi
if [[ -n $baud ]]; then mode="UART at $baud baud, through safeboot"; else mode="USB, through safeboot"; fi
flags=(-b "${baud:-9600}:9600:115200" -s "$safeboot" -t 1 --usb-alt 0)
echo "  mode: $mode"
if systemctl is-active --quiet ModemManager 2>/dev/null &&
	[[ $(udevadm info -q property -n "$dev" 2>/dev/null) != *ID_MM_DEVICE_IGNORE=1* ]]; then
	warn "ModemManager is running and may probe the receiver when it re-enumerates;"
	warn "consider 'sudo systemctl stop ModemManager' for the update"
fi

winbin=$(WINEDEBUG=-all winepath -w "$bin" 2>/dev/null)
args=(-p STDIO "${flags[@]}" --no-fis 1 -v 1 -R 1 "${extra[@]}" "$winbin")
echo "  command: wine $UBXFWUPDATE ${args[*]}"
echo "  relay: ubxfwupdate stdin/stdout <-> $link"
((dry)) && { echo "Dry run, nothing flashed."; exit 0; }

if ((!yes)); then
	read -r -p "Flash firmware $name to $dev? [y/N] " ans || ans=
	[[ $ans == [yY]* ]] || { echo "Cancelled."; exit 1; }
fi

log=${TMPDIR:-/tmp}/f9p_fwupdate_$(date +%Y%m%d_%H%M%S).log
stage 3 "Flashing (started $(date +%T)); don't unplug the receiver or stop this"
t0=$SECONDS
set +e
python3 -I - "$link" "${baud:-115200}" "$log" "$UBXFWUPDATE" "${args[@]}" <<'PY'
import os, re, subprocess, sys, threading, time
import serial

link, baud, log_path, exe, *args = sys.argv[1:]
TAG = re.compile(r"<(?:INF|AC)>([A-Za-z_]+) ?(-?\d*) ?(-?\d*)<\\(?:INF|AC)>")
TTY = sys.stdout.isatty()
log_file = open(log_path, "a", encoding="latin-1", buffering=1)
log_lock = threading.Lock()
out_lock = threading.Lock()
t0 = time.time()


class Progress:
    """log lines pass through; the <INF> tags u-center draws its progress bar from become one status line"""

    def __init__(self):
        self.n = dict.fromkeys(("ERASE", "ERASE_ACK", "WRITE", "WRITE_ACK", "WRITE_AGAIN"), 0)
        self.tot = {"ERASE": None, "WRITE": None}
        self.varies = {"ERASE": False, "WRITE": False}
        self.shown = False
        self.drawn = 0.0
        self.success = False

    def tag(self, name, a, b):
        self.n[name] += 1
        if name in self.tot and not self.varies[name]:
            # the second number is taken as a total only while it never changes
            if self.tot[name] is None and b > a:
                self.tot[name] = b
            elif self.tot[name] is not None and b != self.tot[name]:
                self.tot[name], self.varies[name] = None, True
        self.draw()

    def status(self):
        n, tot = self.n, self.tot
        s = f"erase {n['ERASE_ACK']}" + (f"/{tot['ERASE']}" if tot["ERASE"] else "") + f" sectors | write {n['WRITE_ACK']}"
        if tot["WRITE"]:
            pct = min(100, n["WRITE_ACK"] * 100 // tot["WRITE"])
            s += f"/{tot['WRITE']} acked {pct}% [{'#' * (pct // 5):<20}]"
        else:
            s += f" acked ({n['WRITE']} sent)"
        el = int(time.time() - t0)
        return s + f" | retries {n['WRITE_AGAIN']} | {el // 60}:{el % 60:02d}"

    def draw(self, force=False):
        now = time.time()
        if not force and now - self.drawn < (0.1 if TTY else 10):
            return
        self.drawn = now
        with out_lock:
            if TTY:
                sys.stdout.write("\r\033[K  " + self.status())
            else:
                sys.stdout.write("  " + self.status() + "\n")
            sys.stdout.flush()
            self.shown = True

    def line(self, text):
        with out_lock:
            if TTY and self.shown:
                sys.stdout.write("\r\033[K")
            sys.stdout.write("  " + text + "\n")
            if TTY and self.shown:
                sys.stdout.write("  " + self.status())
            sys.stdout.flush()

    def finish(self):
        if any(self.n.values()):
            self.draw(force=True)
            if TTY:
                print()


def log(text):
    with log_lock:
        log_file.write(text)


def say(text):
    log(f"[relay {time.time() - t0:6.2f}] {text}\n")
    prog.line(text)


class Trace:
    """writes a one-line summary of each UBX frame in one direction to the log"""

    def __init__(self, tag):
        self.tag, self.buf, self.other = tag, b"", 0

    def feed(self, data):
        if self.tag == "rx" and data == b"\0":
            return  # keepalive, not from the receiver
        self.buf += data
        while True:
            i = self.buf.find(b"\xb5\x62")
            if i < 0:
                keep = 1 if self.buf.endswith(b"\xb5") else 0
                self.other += len(self.buf) - keep
                self.buf = self.buf[len(self.buf) - keep:]
                return
            self.other += i
            if len(self.buf) < i + 6:
                self.buf = self.buf[i:]
                return
            n = self.buf[i + 4] | self.buf[i + 5] << 8
            if n > 8192:  # not a real frame
                self.other += 2
                self.buf = self.buf[i + 2:]
                continue
            if len(self.buf) < i + 8 + n:
                self.buf = self.buf[i:]
                return
            f = self.buf[i:i + 8 + n]
            self.buf = self.buf[i + 8 + n:]
            extra = f" (+{self.other} other bytes)" if self.other else ""
            self.other = 0
            log(f"[{self.tag} {time.time() - t0:6.2f}] UBX {f[2]:02X}-{f[3]:02X} len {n}: "
                      f"{f[6:6 + min(n, 24)].hex(' ')}{extra}\n")


class Port:
    """the receiver. After it drops off USB, what the tool writes is thrown away
    until the tool sends <AC>Reenum, as on Windows, where writes to the vanished
    port fail and u-center reopens it on Reenum: stale retries reaching the
    safeboot ROM before the training sequence leave it mute. After Reenum,
    bytes wait in pending until the receiver is back."""

    def __init__(self):
        self.s = None
        self.baud = int(baud)
        self.lock = threading.Lock()
        self.pending = bytearray()
        self.muted = False

    def open(self):
        s = serial.Serial(link, self.baud, timeout=0.02, exclusive=True)
        with self.lock:
            self.s = s
            self._flush()

    def _flush(self):
        if self.s and self.pending:
            try:
                self.s.write(self.pending)
                self.pending.clear()
            except (serial.SerialException, OSError):
                self._close()

    def _close(self):
        try:
            self.s.close()
        except Exception:
            pass
        self.s = None

    def write(self, data):
        """-> what happened to data: sent, held or discarded"""
        with self.lock:
            if self.muted:
                return "discarded"
            self.pending += data
            self._flush()
            return "sent" if self.s else "held"

    def drop(self):
        with self.lock:
            if self.s:
                self._close()
            self.muted = True
            self.pending.clear()

    def unmute(self):
        with self.lock:
            self.muted = False

    def setbaud(self, b):
        with self.lock:
            self.baud = b
            if self.s:
                try:
                    self.s.baudrate = b
                except (serial.SerialException, OSError, ValueError):
                    self._close()


def to_tool(data):
    view = memoryview(data)
    while view:
        view = view[os.write(tool.stdin.fileno(), view):]


def pump_in():
    """receiver -> tool. Under wine the tool's reads block until bytes arrive, and
    it only checks its timeouts and sends its next packet when a read returns, so
    a 0x00 (skipped like any non-UBX byte) goes in after 25 ms of silence. At
    250 ms the flash write ran at 3 packets/s."""
    last = time.time()
    down = back = None
    while tool.poll() is None:
        now = time.time()
        data = b""
        s = port.s
        if s is None:  # off USB: wait for the by-path link to come back
            if down is None:
                down, back = now, None
            if not os.path.exists(link):
                back = None
            elif back is None:
                back = now
            if back is not None and now - back >= 0.5:  # give udev a moment with the new device
                try:
                    port.open()
                    say(f"reconnected after {now - down:.1f} s: {link} -> {os.path.realpath(link)}")
                    down = None
                except (serial.SerialException, OSError):
                    back = now
            time.sleep(0.05)
        else:
            try:
                data = s.read(max(1, s.in_waiting))
            except Exception:
                port.drop()
                if tool.poll() is None:
                    say("receiver dropped off USB, waiting for it")
                continue
        if not data and now - last < 0.025:
            continue
        last = now
        rx.feed(data)
        try:
            to_tool(data or b"\0")
        except OSError:
            return


def pump_out():
    """tool -> receiver"""
    while data := os.read(tool.stdout.fileno(), 65536):
        tx.feed(data)
        what = port.write(data)
        if what != "sent":
            log(f"[relay {time.time() - t0:6.2f}] {what} {len(data)} bytes (receiver dropped off USB)\n")


def pump_log():
    """tool's log on stderr: tags carry no newline, so split them out"""
    buf = ""
    while data := os.read(tool.stderr.fileno(), 4096):
        text = data.decode("latin-1")
        log(text)
        buf += text.replace("\r", "")
        while True:
            m = TAG.search(buf)
            nl = buf.find("\n")
            if m and (nl < 0 or m.start() < nl):
                if buf[:m.start()].strip():
                    prog.line(buf[:m.start()].rstrip())
                name, a, b = m[1], int(m[2] or 0), int(m[3] or 0)
                if name == "Setbaud":
                    port.setbaud(a)
                    say(f"baudrate -> {a}")
                elif name == "Reenum":
                    # u-center reopens its port here; the relay already follows every drop
                    port.unmute()
                    say(f"tool asks to reconnect after re-enumeration ({a}): "
                        + ("connected" if port.s else "still waiting for the receiver"))
                elif name in prog.n:
                    prog.tag(name, a, b)
                else:
                    prog.line(m[0])
                buf = buf[m.end():]
            elif nl >= 0:
                text = buf[:nl].rstrip()
                if "Firmware Update SUCCESS" in text:
                    prog.success = True
                if text.startswith("wine: Unhandled page fault") and prog.success:
                    text = "(wine crashed as the tool exited, after SUCCESS: harmless)"
                if text.strip():
                    prog.line(text)
                buf = buf[nl + 1:]
            else:
                break
    if buf.strip():
        prog.line(buf.strip())


prog, port = Progress(), Port()
rx, tx = Trace("rx"), Trace("tx")
try:
    port.open()
except (serial.SerialException, OSError) as e:
    sys.exit(f"ERROR: can't open {link}: {e}")
tool = subprocess.Popen(["wine", exe, *args], cwd=os.path.dirname(exe), stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, env={**os.environ, "WINEDEBUG": "-all"})
threads = [threading.Thread(target=f, daemon=True) for f in (pump_in, pump_out, pump_log)]
for t in threads:
    t.start()
try:
    rc = tool.wait()
except KeyboardInterrupt:
    tool.wait()
    rc = 130
for t in threads[1:]:
    t.join(timeout=2)
prog.finish()
sys.exit(rc)
PY
rc=$?
set -e

if ((rc == 0)); then
	stage 4 "Done in $(mmss $((SECONDS - t0)))"
	echo "  The receiver resets into the new firmware. The flash was erased, so it runs the"
	echo "  factory default config now: load the right 'F9P ... .txt' config file again."
else
	stage 4 "FAILED (exit $rc$( ((rc == 2)) && echo ': update failed' ))"
	if grep -q 'ERASE' "$log" 2>/dev/null; then
		echo "  Erasing had started, so the receiver has no valid firmware: run the script again"
		echo "  now. It finds the receiver in safeboot and carries on."
	else
		echo "  Nothing was erased. If the receiver now sends no NMEA, it's in safeboot: running"
		echo "  the script again carries on from there, or power-cycle it to get the old firmware back."
	fi
fi
echo "  log: $log"
exit "$rc"
