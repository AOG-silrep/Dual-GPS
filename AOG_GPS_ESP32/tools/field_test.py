#!/usr/bin/env python3
"""Record and check the logs for FIELD_TESTS.md.

With "Debug raw" switched on (web UI, USB Serial debug tab) the ESP32 prints one
block per GPS epoch on its USB serial port, ending with the $PAOGI sentence that
goes to AgOpenGPS. This script records that stream and turns it into the numbers
the field tests ask for.

  field_test.py capture PORT NAME [--minutes M] [--baud B] [--raw]
  field_test.py summary LOG...          tests 1, 2, 3 and 7
  field_test.py passes LOG...           tests 4 and 6
  field_test.py reverse LOG...          test 5
  field_test.py hp-enable PORT [--off]  test 8, on the position F9P's USB port
  field_test.py hpposllh LOG            test 8

capture and hp-enable need pyserial; the rest only need Python 3.
"""

import argparse
import math
import re
import statistics
import struct
import sys
import time

# ---------------------------------------------------------------- parsing

#core log lines ("[435447][V][AsyncTCP.cpp:1138] ...") and the serial readers' prints land in the middle of an epoch's block
LOG_LINE = re.compile(r'\[\s*\d+\]\[[VDIWE]\]\[[^\]\r\n]*\][^\r\n]*\r?\n')
SERIN = re.compile(r'SerIn: (?:RelPosNED heading down dist flags,-?\d+,-?[\d.]+,-?\d+,-?\d+,'
                   r'|#lstUBX #newUBX newUBX lat lon,\d+,\d+,-?\d+,-?\d+,)')

#one per epoch, printed right after the $PAOGI went out
ANCHOR = re.compile(r'millis,(\d+),UBXRingCount1 OGIfromUBX PAOGI,\d+,\d+,')
#two tasks printing at once interleave even inside a number (Arduino prints a float a digit at a time), so
#every value must have its exact print format and each group must be followed by the label that always
#comes next; a garbled group then reads as missing instead of as a wrong number
F2 = r'(-?\d{1,4}\.\d{2})'#Serial.print( double )
#printed after the anchor
FLAGS = re.compile(r'DualGPSPres RollPres VirtAntPres DrivDir FilterPos,(\d),(\d),(\d),(\d),(\d),PVThead')
PVTHEAD = re.compile(r'PVThead RelPosNEDhead,(-?\d{1,8}),(-?\d{1,8}),PVTlat')
OGI = re.compile(r'\$PAOGI,([^*\r\n$]*)\*([0-9A-Fa-f]{2})')
#printed before the anchor
MIX = re.compile(r'VTGQuota HeadQualFac HeadingVTG HeadingRelPos HeadingMixUnLim,' + ','.join([F2] * 5)
                 + ',NoRollCount')
NOROLL = re.compile(r'NoRollCount DualAntNoValueCount,(\d{1,3}),(\d{1,3}),(?=[,rl])')
ANTDIST = re.compile(r'AntDist AntDisGPS,' + F2 + r',(\d{1,4}),headVTG')

OGI_FORMAT = (re.compile(r'^\d{4}\.\d{7}$'), re.compile(r'^\d{5}\.\d{7}$'),
              re.compile(r'^\d{3}\.\d{2}$'), re.compile(r'^-?\d{2}\.\d{2}$'))

KNOT_KMH = 1.852


class Ogi:
    """the fields of one $PAOGI sentence"""
    __slots__ = ('body', 'tod', 'lat', 'lon', 'fix', 'sats', 'speed', 'head', 'roll')


class Epoch:
    """one epoch of the debug raw stream; None where that part could not be parsed"""
    __slots__ = ('ms', 'ogi', 'ogi_bad', 'dual', 'rollp', 'virt', 'drv', 'fpos', 'head_mot',
                 'quota', 'qual', 'head_vtg', 'head_dual', 'head_mix', 'no_roll', 'dnv',
                 'ant_dist', 'rel_len')

    def __init__(self, ms):
        for name in self.__slots__:
            setattr(self, name, None)
        self.ms = ms
        self.ogi_bad = False


def checksum_ok(body, cs):
    c = 0
    for ch in 'PAOGI,' + body:
        c ^= ord(ch)
    return '%02X' % c == cs.upper()


def parse_ogi(body):
    f = body.split(',')
    if len(f) < 13 or not re.match(r'^\d$', f[5]):
        return None
    try:
        o = Ogi()
        o.body = body
        o.tod = int(f[0][0:2]) * 3600 + int(f[0][2:4]) * 60 + float(f[0][4:])
        o.lat = int(f[1][:2]) + float(f[1][2:]) / 60
        o.lon = int(f[3][:3]) + float(f[3][3:]) / 60
        if f[2] == 'S':
            o.lat = -o.lat
        if f[4] == 'W':
            o.lon = -o.lon
        o.fix = f[5]
        o.sats = int(f[6])
        o.speed = float(f[10])#knots, negative when the firmware thinks it is reversing
        o.head = float(f[11])
        o.roll = float(f[12])
    except (ValueError, IndexError):
        return None
    return o if 0 <= o.head < 360 else None


def last_match(rx, s):
    m = None
    for m in rx.finditer(s):
        pass
    return m


def load(path):
    text = open(path, 'rb').read().decode('latin-1')
    text = SERIN.sub('', LOG_LINE.sub('', text))
    anchors = list(ANCHOR.finditer(text))
    epochs = []
    for k, a in enumerate(anchors):
        #before the anchor: this epoch's heading prints; after it: its flags and $PAOGI
        pre = text[anchors[k - 1].end() if k else 0:a.start()]
        post = text[a.end():anchors[k + 1].start() if k + 1 < len(anchors) else len(text)]
        e = Epoch(int(a.group(1)))
        m = FLAGS.search(post)
        if m:
            e.dual, e.rollp, e.virt, e.drv, e.fpos = map(int, m.groups())
        m = PVTHEAD.search(post)
        if m and all(0 <= int(v) <= 36000000 for v in m.groups()):
            e.head_mot = int(m.group(1)) * 1e-5
        m = OGI.search(post)
        if m:
            if checksum_ok(m.group(1), m.group(2)):
                e.ogi = parse_ogi(m.group(1))
            e.ogi_bad = e.ogi is None
        m = last_match(MIX, pre)
        if m and all(0 <= float(v) < 360 for v in m.groups()[2:]):
            e.quota, e.qual, e.head_vtg, e.head_dual, e.head_mix = map(float, m.groups())
        m = last_match(NOROLL, pre)
        if m:
            e.no_roll, e.dnv = int(m.group(1)), int(m.group(2))
        m = last_match(ANTDIST, pre)
        #only printed when the length passed the outer band, at most a factor 1.99 off
        if m and float(m.group(1)) / 2 < int(m.group(2)) < float(m.group(1)) * 2:
            e.ant_dist, e.rel_len = float(m.group(1)), int(m.group(2))
        epochs.append(e)
    #DualAntNoValueCount counts up by one per very poor epoch (held at 220) and is 0 in an epoch with a
    #dual heading, so a digit from another task that turned 0 into 70 shows up as a jump
    for k, e in enumerate(epochs):
        if not e.dnv:
            continue
        prev = epochs[k - 1].dnv if k else None
        nxt = epochs[k + 1].dnv if k + 1 < len(epochs) else None
        if e.ant_dist is not None or not (e.dnv == 1 or prev in (e.dnv - 1, e.dnv) or nxt in (e.dnv + 1, e.dnv)):
            e.dnv = None
    if not epochs:
        sys.exit('%s: no debug raw epochs found. Was "Debug raw" on, and is this the ESP32\'s port?' % path)
    return epochs


# ---------------------------------------------------------------- helpers

def adiff(a, b):
    """a - b in (-180, 180]"""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def cmean(angles):
    angles = list(angles)
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(s, c)) % 360.0


def rms(values):
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def pct(part, whole):
    return 100.0 * part / whole if whole else float('nan')


def cardinal(heading):
    return 'NESW'[int(round(heading / 90.0)) % 4]


def epoch_ms(epochs):
    gaps = [b.ms - a.ms for a, b in zip(epochs, epochs[1:]) if 0 < b.ms - a.ms < 5000]
    return statistics.median(gaps) if gaps else 100


def duration_s(epochs):
    return sum(b.ms - a.ms for a, b in zip(epochs, epochs[1:]) if 0 < b.ms - a.ms < 5000) / 1000.0


def metres_per_degree(lat0):
    """(north, east) metres per degree of latitude/longitude at lat0, WGS84"""
    a, e2 = 6378137.0, 6.69437999014e-3
    s = math.sin(math.radians(lat0))
    w = 1.0 - e2 * s * s
    return (math.radians(1) * a * (1.0 - e2) / w ** 1.5,
            math.radians(1) * a / math.sqrt(w) * math.cos(math.radians(lat0)))


def verdict(test, result, text):
    print('  %-8s %-5s %s' % (test, result, text))


# ---------------------------------------------------------------- capture

def cmd_capture(args):
    import serial
    name = '%s_%s.log' % (args.name, time.strftime('%Y%m%d-%H%M%S'))
    port = serial.Serial(args.port, args.baud, timeout=0.2)#DTR/RTS stay asserted, so the ESP32 is not reset
    print('logging %s to %s, Ctrl-C to stop' % (args.port, name))
    start = last = time.time()
    window, total, seen = '', 0, False
    with open(name, 'wb') as f:
        try:
            while args.minutes is None or time.time() - start < args.minutes * 60:
                data = port.read(4096)
                if data:
                    f.write(data)
                    total += len(data)
                    window += data.decode('latin-1')
                now = time.time()
                if now - last < 5:
                    continue
                n = len(ANCHOR.findall(window))
                line = '%5.0f s %7.0f kB' % (now - start, total / 1000.0)
                if n:
                    seen = True
                    line += ' %4.1f epochs/s' % (n / (now - last))
                    o = OGI.findall(window)
                    fl = FLAGS.findall(window)
                    if o:
                        g = o[-1][0].split(',')
                        if len(g) > 12:
                            line += '  fix %s  %5.1f km/h  heading %s' % (g[5], float(g[10] or 0) * KNOT_KMH, g[11])
                    if fl:
                        line += '  dual %s filt %s dir %s' % (fl[-1][0], fl[-1][4], fl[-1][3])
                elif not args.raw and not seen:
                    line += '  no debug raw output yet: switch "Debug raw" on (USB Serial debug tab)'
                print(line, flush=True)
                window, last = window[-300:], now
        except KeyboardInterrupt:
            pass
    port.close()
    print('saved %s (%.0f kB)' % (name, total / 1000.0))


# ---------------------------------------------------------------- summary: tests 1, 2, 3, 7

def cmd_summary(args):
    for path in args.logs:
        eps = load(path)
        step = epoch_ms(eps)
        dur = duration_s(eps)
        good = [e for e in eps if e.ogi]
        rtk = [e for e in good if e.ogi.fix == '4']
        print('\n%s' % path)
        print('  %d epochs, %.0f s (%.1f min), one every %.0f ms; %d with a valid $PAOGI, %d of them RTK fixed'
              % (len(eps), dur, dur / 60, step, len(good), len(rtk)))
        fixes = {}
        for e in good:
            fixes[e.ogi.fix] = fixes.get(e.ogi.fix, 0) + 1
        print('  fix quality: ' + ', '.join('%s: %d' % (k, v) for k, v in sorted(fixes.items()))
              + '   (4 = RTK fixed, 5 = RTK float)')

        #test 1: antenna distance, on the epochs that had a new dual heading
        f = args.factor
        dual = [e for e in eps if e.ant_dist is not None]
        print('\n  Test 1, antenna distance (deviation factor %.2f)' % f)
        if dual:
            devs = sorted(e.rel_len - e.ant_dist for e in dual)
            band = [e for e in dual if e.ant_dist / (f / 4 + 0.75) < e.rel_len < e.ant_dist * (f / 4 + 0.75)]
            settings = sorted({e.ant_dist for e in dual})
            share = pct(len(band), len(dual))
            print('    AntDist setting(s): %s cm; measured - setting: median %+.0f cm, 5..95%% %+.0f..%+.0f cm'
                  % (', '.join('%.0f' % s for s in settings), statistics.median(devs),
                     devs[int(len(devs) * 0.05)], devs[int(len(devs) * 0.95)]))
            print('    %.1f%% of dual epochs inside the "perfect" band (+-%.0f%%)' % (share, (f / 4 - 0.25) * 100))
            if share >= 99:
                verdict('Test 1', 'PASS', 'antenna distance matches the setting')
            else:
                tip = ''
                if len(settings) == 1:
                    tip = '; if it is a steady offset, set the distance to %.0f cm' % (settings[0] + statistics.median(devs))
                verdict('Test 1', 'FAIL' if share < 95 else 'WARN', '%.1f%% in band%s' % (share, tip))
        else:
            print('    no epoch had a new dual heading')
            verdict('Test 1', 'FAIL', 'no dual heading in this log')

        #which branch headingRollCalc took; DualAntNoValueCount is reset in the perfect and medium branches
        parsed = [e for e in eps if e.dnv is not None]
        poor = [e for e in parsed if e.dnv > 0]
        medium = len(dual) - (len(band) if dual else 0)
        single = sum(1 for e in parsed if e.dnv == 0 and e.ant_dist is None and e.dual == 0)
        print('    branch: perfect %d, medium %d, very poor %d, no dual heading %d, garbled in the log %d'
              % (len(band) if dual else 0, medium, len(poor), single,
                 len(eps) - len(dual) - len(poor) - single))

        #test 2: RelPosNED stale or invalid -> very poor branch
        print('\n  Test 2, RelPosNED on time')
        runs, cur = [], 0
        for e in parsed:
            if e.dnv > 0:
                cur += 1
            elif cur:
                runs.append(cur)
                cur = 0
        if cur:
            runs.append(cur)
        share = pct(len(poor), len(parsed))
        hist = {}
        for r in runs:
            key = r if r < 5 else '5+'
            hist[key] = hist.get(key, 0) + 1
        print('    very poor branch: %d epochs (%.2f%%) in %d runs, run lengths %s'
              % (len(poor), share, len(runs), dict(sorted(hist.items(), key=lambda kv: str(kv[0])))))
        if not poor:
            verdict('Test 2', 'PASS', 'no epoch lost its dual heading')
        elif share <= 0.1:
            verdict('Test 2', 'PASS', 'only %d epochs; check the web UI counters as in FIELD_TESTS.md' % len(poor))
        else:
            verdict('Test 2', 'CHECK', 'compare "Stale at NAV-PVT" with the "Dropped" rows in the web UI (FIELD_TESTS.md test 2)')

        #test 3: filtered position sent
        print('\n  Test 3, position sent without the filter')
        filt = [e for e in rtk if e.fpos == 1]
        share = pct(len(filt), len(rtk))
        print('    FilterPos=1 on %d of %d RTK fixed epochs (%.2f%%)' % (len(filt), len(rtk), share))
        if filt:
            why_medium = sum(1 for e in filt if e.ant_dist is not None)
            why_poor = sum(1 for e in filt if (e.dnv or 0) > 0)
            print('    of those: medium band %d (test 1), very poor %d (test 2), other %d'
                  % (why_medium, why_poor, len(filt) - why_medium - why_poor))
        if not rtk:
            verdict('Test 3', 'FAIL', 'no RTK fixed epochs in this log')
        else:
            verdict('Test 3', 'PASS' if share < 1 else 'WARN' if share < 5 else 'FAIL',
                    '%.2f%% of RTK fixed epochs sent the filtered position' % share)

        #test 7: $PAOGI format
        print('\n  Test 7, $PAOGI format')
        broken = sum(1 for e in eps if e.ogi_bad)
        badfmt = 0
        for e in good:
            g = e.ogi.body.split(',')
            if not (OGI_FORMAT[0].match(g[1]) and OGI_FORMAT[1].match(g[3]) and OGI_FORMAT[2].match(g[11])
                    and float(g[11]) < 360 and OGI_FORMAT[3].match(g[12])):
                badfmt += 1
        print('    %d valid, %d broken in the log (other prints mixed in, not a fault of the UDP output), '
              '%d with a wrong field format' % (len(good), broken, badfmt))
        #a wrong checksum in the firmware would break every sentence, the log only breaks some
        ok = good and badfmt == 0 and len(good) > broken
        verdict('Test 7', 'PASS' if ok else 'FAIL',
                'lat/lon ddmm.mmmmmmm, heading ddd.dd, roll dd.dd' if ok else 'see the counts above')


# ---------------------------------------------------------------- reverse: test 5

def cmd_reverse(args):
    events = []
    for path in args.logs:
        eps = load(path)
        step = epoch_ms(eps)
        use = [e for e in eps if e.ogi and e.head_mot is not None and e.head_dual is not None and e.drv is not None]
        moving = lambda e: abs(e.ogi.speed) > 0.2#0.37 km/h, like the firmware's 100 mm/s
        back = [moving(e) and abs(adiff(e.head_mot, e.head_dual)) > 90 for e in use]
        fwd = [moving(e) and not b for e, b in zip(use, back)]
        guard = set()
        for i in range(len(use) - 3):
            #starts moving backwards: 3 epochs in a row, none in the 10 before
            if all(back[i:i + 3]) and not any(back[max(0, i - 10):i]):
                j = next((k for k in range(i, min(i + 60, len(use))) if use[k].drv == 2), None)
                wander = max(abs(adiff(use[k].ogi.head, use[i - 1].ogi.head))
                             for k in range(i, min(i + int(2000 / step), len(use)))) if i else float('nan')
                #back to forward
                end = next((k for k in range(i + 3, len(use) - 3) if all(fwd[k:k + 3])), None)
                resume = None
                if end is not None:
                    r = next((k for k in range(end, min(end + 60, len(use))) if use[k].drv != 2), None)
                    resume = None if r is None else use[r].ms - use[end].ms
                    guard.update(range(end, end + 20))
                events.append((path, use[i], None if j is None else j - i,
                               None if j is None else use[j].ms - use[i].ms, wander, resume))
        false_back = sum(1 for k, e in enumerate(use) if fwd[k] and abs(e.ogi.speed) > 1.0 and e.drv == 2
                         and k not in guard)
        if false_back:
            verdict('Test 5', 'FAIL', '%s: %d epochs flagged backwards while driving forward' % (path, false_back))
    if not events:
        print('no reversal found (needs 3+ epochs moving backwards faster than 0.4 km/h)')
        return
    print('\n%-10s %6s %10s %10s %13s %12s' % ('GPS time', 'facing', 'detected', 'after', 'heading moved', 'forward again'))
    worst = 0
    for path, e, n, ms, wander, resume in events:
        t = e.ogi.tod
        stamp = '%02d:%02d:%04.1f' % (t // 3600, t % 3600 // 60, t % 60)
        if n is None:
            worst = 99
            print('%-10s %6s %10s' % (stamp, cardinal(e.head_dual), 'never'))
            continue
        worst = max(worst, n)
        print('%-10s %6s %4d epochs %7d ms %10.2f deg %12s'
              % (stamp, cardinal(e.head_dual), n, ms, wander, '-' if resume is None else '%d ms' % resume))
    big = max(ev[4] for ev in events if not math.isnan(ev[4]))
    verdict('Test 5', 'PASS' if worst <= 7 else 'WARN' if worst <= 10 else 'FAIL',
            'slowest detection %s epochs (expect 1-6)' % ('never' if worst == 99 else worst))
    verdict('Test 5', 'PASS' if big < 2 else 'WARN', 'heading moved at most %.2f deg in the first 2 s of a reversal' % big)


# ---------------------------------------------------------------- passes: tests 4 and 6

def find_passes(eps, step):
    seg, segs = [], []
    for e in eps:
        if not e.ogi:#garbled in the log, the tractor did not stop
            continue
        if e.ogi.speed <= 1.0:#forward, faster than 1.9 km/h
            if seg:
                segs.append(seg)
            seg = []
            continue
        if seg and not 0 < e.ms - seg[-1].ms <= 1000:
            segs.append(seg)
            seg = []
        seg.append(e)
    if seg:
        segs.append(seg)
    w = max(1, int(2000 / step))#straight over +-2 s
    out = []
    for s in segs:
        h = [e.ogi.head for e in s]
        straight = [w <= i < len(s) - w and abs(adiff(h[i - w], h[i])) < 2 and abs(adiff(h[i + w], h[i])) < 2
                    for i in range(len(s))]
        run = []
        for i, e in enumerate(s):
            if straight[i] and (not run or abs(adiff(h[i], run[0].ogi.head)) < 5):
                run.append(e)
                continue
            if run:
                out.append(run)
            run = [e] if straight[i] else []
        if run:
            out.append(run)
    return [p for p in out if (p[-1].ms - p[0].ms) >= 15000]


def pass_stats(p, step):
    lat0 = p[0].ogi.lat
    mn, me = metres_per_degree(lat0)
    pts = [((e.ogi.lon - p[0].ogi.lon) * me, (e.ogi.lat - lat0) * mn) for e in p]
    cx = sum(x for x, _ in pts) / len(pts)
    cy = sum(y for _, y in pts) / len(pts)
    sxx = sum((x - cx) ** 2 for x, _ in pts)
    syy = sum((y - cy) ** 2 for _, y in pts)
    sxy = sum((x - cx) * (y - cy) for x, y in pts)
    th = 0.5 * math.atan2(2 * sxy, sxx - syy)
    along = [(x - cx) * math.cos(th) + (y - cy) * math.sin(th) for x, y in pts]
    lat = [-(x - cx) * math.sin(th) + (y - cy) * math.cos(th) for x, y in pts]
    h = [e.ogi.head for e in p]
    #second differences: smooth steering motion cancels, epoch-to-epoch jitter stays (sqrt(6) x its RMS)
    triples = [i for i in range(1, len(p) - 1) if p[i + 1].ms - p[i - 1].ms <= 3 * step]
    head = cmean(h)
    nominal = round(head / 90.0) * 90 % 360
    side = [adiff(x, nominal) for x in h]
    side = [x for x in side if x != 0]
    return dict(
        head=head, axis='N-S' if nominal in (0, 180) else 'E-W',
        length=max(along) - min(along), secs=(p[-1].ms - p[0].ms) / 1000.0,
        kmh=statistics.mean(e.ogi.speed for e in p) * KNOT_KMH,
        lat_sd=statistics.pstdev(lat) * 100, lat_max=max(abs(v) for v in lat) * 100,
        lat_noise=rms(lat[i + 1] - 2 * lat[i] + lat[i - 1] for i in triples) / math.sqrt(6) * 100,
        head_noise=rms(adiff(h[i + 1], h[i]) - adiff(h[i], h[i - 1]) for i in triples) / math.sqrt(6),
        crossings=sum(1 for x, y in zip(side, side[1:]) if (x < 0) != (y < 0)),
        filt=pct(sum(1 for e in p if e.fpos == 1), len(p)),
        poor=pct(sum(1 for e in p if (e.dnv or 0) > 0), len(p)),
        t=p[0].ogi.tod)


def cmd_passes(args):
    stats = []
    for path in args.logs:
        eps = load(path)
        step = epoch_ms(eps)
        stats += [pass_stats(p, step) for p in find_passes(eps, step)]
    if not stats:
        print('no straight pass found (needs 15 s straight within +-2 deg, faster than 1.9 km/h)')
        return
    print('\n%-8s %4s %7s %5s %5s %5s %8s %8s %9s %10s %6s %5s %5s'
          % ('GPS time', 'axis', 'heading', 'm', 's', 'km/h', 'lat sd', 'lat max', 'lat noise', 'head noise',
             'cross', 'filt%', 'poor%'))
    for s in stats:
        t = s['t']
        print('%02d:%02d:%02d %4s %7.2f %5.0f %5.0f %5.1f %5.1f cm %5.1f cm %6.2f cm %6.3f deg %6d %5.1f %5.1f'
              % (t // 3600, t % 3600 // 60, t % 60, s['axis'], s['head'], s['length'], s['secs'], s['kmh'],
                 s['lat_sd'], s['lat_max'], s['lat_noise'], s['head_noise'], s['crossings'], s['filt'], s['poor']))
    print('\n  lat sd/max = position sent to AOG, sideways from the best straight line (steering + GPS)')
    print('  lat noise, head noise = epoch-to-epoch jitter of the position and heading sent to AOG')
    print('  cross = times the heading crossed the line\'s axis (0, 90, 180 or 270 deg)')
    ns = [s for s in stats if s['axis'] == 'N-S']
    ew = [s for s in stats if s['axis'] == 'E-W']
    if not ns or not ew:
        print('  need passes on both axes for the N-S / E-W comparison')
        return
    mean = lambda rows, key: statistics.mean(r[key] for r in rows)
    print('\n  %-11s %10s %10s %7s' % ('', 'N-S mean', 'E-W mean', 'ratio'))
    for key, label in (('head_noise', 'head noise'), ('lat_noise', 'lat noise'), ('lat_sd', 'lat sd')):
        print('  %-11s %10.3f %10.3f %7.2f' % (label, mean(ns, key), mean(ew, key), mean(ns, key) / mean(ew, key)))
    print('  (lat sd also depends on the steering tune and the ground)')
    #test 4: the old cos filter flipped the heading by +- the noise every time it crossed 0 or 180
    cross = sum(s['crossings'] for s in ns)
    r = mean(ns, 'head_noise') / mean(ew, 'head_noise')
    if cross < 10:
        verdict('Test 4', 'INCONCLUSIVE', 'N-S passes crossed 0/180 deg only %d times; '
                'set the AB line by heading at exactly 0.0 deg' % cross)
    else:
        verdict('Test 4', 'PASS' if r <= 1.33 else 'WARN' if r <= 1.67 else 'FAIL',
                'N-S heading noise %.2fx E-W, with %d crossings of 0/180 deg (about 2x before the fix)' % (r, cross))
    r = mean(ns, 'lat_noise') / mean(ew, 'lat_noise')
    worst = max(r, 1 / r)
    verdict('Test 6', 'PASS' if worst <= 1.33 else 'WARN' if worst <= 1.67 else 'FAIL',
            'N-S position noise %.2fx E-W' % r)


# ---------------------------------------------------------------- HPPOSLLH: test 8

def ubx_frame(cls, mid, payload):
    body = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b'\xb5\x62' + body + bytes([a, b])


def ubx_messages(data):
    i = 0
    while True:
        i = data.find(b'\xb5\x62', i)
        if i < 0 or i + 8 > len(data):
            return
        length = struct.unpack_from('<H', data, i + 4)[0]
        end = i + 8 + length
        if end <= len(data) and ubx_frame(data[i + 2], data[i + 3], data[i + 6:end - 2]) == data[i:end]:
            yield data[i + 2], data[i + 3], data[i + 6:end - 2]
            i = end
        else:
            i += 2


def cmd_hp_enable(args):
    import serial
    key = 0x20910036#CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB
    value = 0 if args.off else 1
    msg = ubx_frame(0x06, 0x8A, struct.pack('<BBHIB', 0, 0x01, 0, key, value))#CFG-VALSET, RAM layer only
    port = serial.Serial(args.port, 115200, timeout=0.2)
    port.reset_input_buffer()
    port.write(msg)
    data, end = b'', time.time() + 3
    while time.time() < end:
        data += port.read(4096)
        for cls, mid, payload in ubx_messages(data):
            if cls == 0x05 and payload[:2] == b'\x06\x8a':
                port.close()
                if mid == 0x01:
                    print('NAV-HPPOSLLH on USB %s (RAM only, a power cycle undoes it)' % ('off' if args.off else 'on'))
                    return
                sys.exit('the receiver refused the setting (ACK-NAK)')
    port.close()
    sys.exit('no answer from the receiver; is %s the position F9P\'s USB port?' % args.port)


def cmd_hpposllh(args):
    data = open(args.log, 'rb').read()
    rows = []
    for cls, mid, p in ubx_messages(data):
        if cls == 0x01 and mid == 0x14 and len(p) == 36:
            _, _, flags, itow, lon, lat, _, _, lon_hp, lat_hp, _, _, h_acc, _ = struct.unpack('<B2sBIiiiibbbbII', p)
            if not flags & 1:
                rows.append((itow, lat, lon, lat_hp, lon_hp, h_acc))
    if len(rows) < 300:
        sys.exit('only %d NAV-HPPOSLLH messages (need 300); was it enabled on this port at rate 1?' % len(rows))
    gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:]) if b[0] > a[0]]
    step = statistics.median(gaps)
    lat0 = rows[0][1] * 1e-7
    mn, me = metres_per_degree(lat0)
    north = [((r[1] - rows[0][1]) * 1e-7 + r[3] * 1e-9) * mn for r in rows]
    east = [((r[2] - rows[0][2]) * 1e-7 + r[4] * 1e-9) * me for r in rows]
    grid_n = [(r[1] - rows[0][1]) * 1e-7 * mn for r in rows]
    ok = [i for i in range(1, len(rows)) if rows[i][0] - rows[i - 1][0] == step]
    noise_n = statistics.pstdev([north[i] - north[i - 1] for i in ok]) / math.sqrt(2) * 1000
    noise_e = statistics.pstdev([east[i] - east[i - 1] for i in ok]) / math.sqrt(2) * 1000
    grid_err = rms(g - n for g, n in zip(grid_n, north)) * 1000
    steps = pct(sum(1 for i in ok if grid_n[i] != grid_n[i - 1]), len(ok))
    print('\n%s: %d epochs, one every %d ms, hAcc median %.1f mm'
          % (args.log, len(rows), step, statistics.median(r[5] for r in rows) / 10.0))
    if step > 200:
        print('  WARN: more than 200 ms between messages; set the USB rate to 1 or the noise is hidden')
    print('  epoch-to-epoch noise: north %.1f mm, east %.1f mm' % (noise_n, noise_e))
    print('  NAV-PVT 1e-7 deg grid: %.1f mm north-south; rounding error %.1f mm RMS; the grid value changes on %.0f%% of epochs'
          % (1e-7 * mn * 1000, grid_err, steps))
    noise = min(noise_n, noise_e)
    if noise < 5:
        verdict('Test 8', 'DO', 'noise below the 11 mm grid: brief 5 would remove a visible staircase')
    elif noise < 10:
        verdict('Test 8', 'MAYBE', 'noise about half the grid: brief 5 would help a little')
    else:
        verdict('Test 8', 'SKIP', 'the noise hides the grid: brief 5 would not change much')


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('capture', help='record a port to NAME_<date>-<time>.log')
    p.add_argument('port')
    p.add_argument('name')
    p.add_argument('--minutes', type=float)
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--raw', action='store_true', help='not a debug raw stream (F9P USB for test 8)')
    p.set_defaults(func=cmd_capture)
    p = sub.add_parser('summary', help='tests 1, 2, 3 and 7')
    p.add_argument('logs', nargs='+')
    p.add_argument('--factor', type=float, default=1.2, help='"Distance between Antennas max Deviation" setting')
    p.set_defaults(func=cmd_summary)
    for name, func, text in (('passes', cmd_passes, 'tests 4 and 6'), ('reverse', cmd_reverse, 'test 5')):
        p = sub.add_parser(name, help=text)
        p.add_argument('logs', nargs='+')
        p.set_defaults(func=func)
    p = sub.add_parser('hp-enable', help='test 8: NAV-HPPOSLLH on the F9P USB port, RAM only')
    p.add_argument('port')
    p.add_argument('--off', action='store_true')
    p.set_defaults(func=cmd_hp_enable)
    p = sub.add_parser('hpposllh', help='test 8: noise from a NAV-HPPOSLLH log')
    p.add_argument('log')
    p.set_defaults(func=cmd_hpposllh)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
