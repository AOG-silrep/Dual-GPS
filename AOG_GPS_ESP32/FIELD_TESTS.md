# Field tests: precision and steering fixes

These tests check the October 2026 precision changes on the tractor:

| Change | Commit | Checked by |
|---|---|---|
| lat/lon with 7 decimals of minutes | c6cc05c | test 7 |
| heading and roll with 2 decimals, rounded | 23320d1 | test 7 |
| filtered position only on a weak signal | 7b78058 | test 3 |
| UBX readers above the web UI (RELPOSNED read on time) | 6093699 | test 2 |
| heading filtered as a sin/cos pair | 9d87f5e | tests 4 and 5 |

Tests 1 and 6 check the whole system, and test 8 decides whether NAV-HPPOSLLH (brief 5) is
worth doing.

Allow about 1½ hours. Tests 1 and 2 only need the web UI. The others are judged from a log that
`tools/field_test.py` records on a laptop and then checks, printing PASS, WARN or FAIL for each
test.

## What to bring

- An open-sky field with room for straight passes of at least 150 m in both a north-south and an
  east-west direction, and a headland to turn on.
- A laptop with Python 3, pyserial (`python3 -m pip install pyserial`) and a copy of this repo.
- A USB cable from the laptop to the ESP32.
- A phone or tablet with the web UI open. Leave it open during the runs: the web UI's load is
  what used to make RELPOSNED late.
- A tape measure (test 1).
- A pen, and the results sheet at the end of this file.

## Setup (10 min)

1. **Firmware.** Status tab → "Build date" must be 16 Oct 2026 or later, built from commit
   9d87f5e or newer.
2. **Save the settings.** Configurations tab → "Download the config". Keep the file with the logs.
3. **RTK fixed.** Wait until AOG shows RTK fix and the Status tab's "GPS receivers" pane says
   "GPS: Dual".
4. **Clear the counters.** Status tab → "Reset all".
5. **Laptop.** Plug it into the ESP32's USB port and find the port: `ls /dev/ttyUSB*` (it is
   usually `/dev/ttyUSB0` when nothing else is plugged in). Make a folder for today's logs and work
   from it:

   ```
   mkdir -p ~/field-logs/$(date +%F) && cd ~/field-logs/$(date +%F)
   alias ft='python3 ~/AOG/Dual_Gps/AOG_GPS_ESP32/tools/field_test.py'
   ```

6. **Debug raw on.** USB Serial debug tab → "Debug raw" on. It takes effect at once.
   **Do not press "Store the Settings" while it is on**, or it stays on after every restart.
7. **Check the log.** Run `ft capture /dev/ttyUSB0 setup --minutes 0.2`. Every 5 s it should
   print about `10.0 epochs/s  fix 4 ...`. If it says "no debug raw output yet", check step 6 and
   the port.

> **Logging changes the timing a little.** Debug raw almost fills the 115200 baud USB link, and
> its prints run in the same task that sends to AOG. On the bench, the time the output left the
> ESP32 varied by up to 47 ms with logging on. Use the logs for the numbers, but judge steering
> feel with Debug raw **off** (test 6, part B).
>
> Optional, to cut that roughly 8×: in the downloaded config, set `"baudrate"` under
> `"connection"` to `921600`, upload it (Configurations tab, the ESP32 restarts), and add
> `--baud 921600` to every `ft capture`. Set it back to 115200 afterwards.

## Test 1: antenna distance (5 min, web UI only)

**Why.** If the measured distance between the antennas is more than 5% off the setting, every
epoch takes the firmware's "medium" branch. That branch lets roll fade to 0, turns on the
position filter, and mixes single-antenna course into the heading (about 35% at 10 km/h). That
causes more twitch than any of the code fixes.

1. Parked: watch Status → "GPS receivers" → "Antenna dist. deviation" for 1 minute. It is
   measured minus set, in cm.
2. Driving: watch it again for 2 minutes, including a turn.

**Pass:** it stays within ±5% of "Distance between Dual GPS Antennas" (±7 cm at 140 cm).
Usually it sits within ±2 cm.

**If it fails:**
- Steady offset (for example always −12): the setting is wrong. Measure antenna centre to centre,
  or set the GPS tab's distance to the setting plus the typical deviation. Store, then repeat.
- Jumping around: an antenna, cable or mounting problem, not a setting.

The run B log repeats this check over every epoch (`ft summary`, "Test 1").

## Run B: logged passes (25 min), for tests 2, 3, 4, 6 and 7

**Set up two AB lines in AOG by heading, not by driving A and B:**
- one at exactly **0.0°** (north-south)
- one at exactly **90.0°** (east-west)

The heading test needs the tractor's heading to wander back and forth across 0°/180°, and only an
AB line at exactly 0.0° makes that happen. If your AOG version can't take a typed heading, use the
straightest north-south line you have and look at the "cross" column afterwards.

1. Start the log: `ft capture /dev/ttyUSB0 b-drive`
2. With autosteer on, at normal working speed (6 km/h or more), drive:
   - the 0.0° line: north, south, north (3 passes of at least 150 m)
   - the 90.0° line: east, west, east (3 passes)
3. Turn on the headland between passes as usual. Note the speed, implement and ground.
4. Press Ctrl-C to stop the log.

Then run these and copy the PASS/WARN/FAIL lines onto the sheet:

```
ft summary b-drive_*.log
ft passes b-drive_*.log
```

### Test 2: RELPOSNED read on time

**Why.** When NAV-RELPOSNED was read late, about 9% of epochs dropped their dual heading for 1–4
epochs. Commit 6093699 fixed that on the bench.

Read these at the end of runs B and C: Status tab → "RelPosNED (heading receiver)" table →
"Stale at NAV-PVT (> 2 epochs)", plus the "Bad checksum", "Wrong length/version" and "Dropped: …"
rows.

**Pass:** Stale is 0, or no bigger than the Dropped and Bad rows added together. Under open sky
they should all be 0.

**Fail:** Stale grows while every Dropped and Bad row stays at 0. That means the readers are late
again. Note the count, and what the web UI was showing at the time.

`ft summary` reports the matching log figure ("very poor branch … epochs in … runs"). On the
bench before the fix it was about 9%, in runs of 1–4 epochs; after the fix it was 0.

### Test 3: position sent without filter lag

From `ft summary`, "Test 3".

**Pass:** FilterPos=1 on under 1% of RTK-fixed epochs. Before commit 7b78058 the filtered position
was sent on every epoch.

If it fails, the same output says why. "Medium band" points to test 1; "very poor" points to
test 2.

### Test 4: heading on the 0° line

**Why.** The old filter only smoothed cos(heading). Every time the heading crossed 0° or 180°, the
heading sent to AOG flipped by ± the noise, about twice the jitter seen on east-west lines.

From `ft passes`, "Test 4". It compares the heading noise on north-south passes with east-west
passes. It only judges if the north-south passes crossed 0°/180° at least 10 times in total (the
"cross" column).

**Pass:** north-south heading noise is at most 1.33× east-west. A simulated drive with the old
filter gave 1.79×; the new one gives 1.00×.

**Inconclusive:** too few crossings. The AB line was not at exactly 0.0°. Redo run B.

### Test 6, part A: steering on north-south vs east-west lines

From `ft passes`, "Test 6", plus the table per pass:
- **lat noise:** epoch-to-epoch sideways jitter of the position sent to AOG.
- **lat sd:** how far the tractor wandered from the best straight line. This depends on the
  steering tune and the ground too.

**Pass:** north-south and east-west position noise are within 1.33× of each other. Write down lat
sd and lat noise for both axes. They are the baseline to compare future changes against.

### Test 7: $PAOGI format

From `ft summary`, "Test 7".

**Pass:** lat/lon have 7 decimals of minutes, heading has 2 decimals and is never 360.00, and roll
has 2 decimals. A few sentences "broken in the log" are normal: other prints get mixed into the
log, not into the UDP output.

Also check that AOG shows no NMEA or GPS errors and the position looks steady.

## Run C: reversing (10 min), for test 5

**Why.** The new heading filter needs a few epochs to notice reversing: 1 epoch at the filter's
medium setting, 6 (0.6 s) at its slow setting, which it uses at low speed. Until then the speed
in the NMEA output keeps a forward sign. Before, the switch was instant, but the heading passed
through fake values on the way.

1. Start the log: `ft capture /dev/ttyUSB0 c-reverse`
2. Six times, three facing north or south and three facing east or west:
   - drive forward about 20 m at 3–5 km/h, straight
   - stop fully for 2 s
   - reverse straight about 10 m at 2–4 km/h
   - stop, then drive forward again
3. Watch AOG while doing it: when does it show reversing, and does the heading or the guidance line
   jump?
4. Press Ctrl-C to stop the log, then run `ft reverse c-reverse_*.log`

**Pass:**
- every reversal is detected within 7 epochs (0.7 s at 10 Hz)
- the heading moves less than 2° in the first 2 s of each reversal
- no epochs are flagged as backwards while driving forward

**If it fails:** note the delays and when they happened. A slow detection at low speed would mean
the filter's slow setting needs another look.

## Run D: steering feel with logging off (15 min), test 6 part B

1. USB Serial debug tab → "Debug raw" **off**.
2. Drive the 0.0° and 90.0° lines again, autosteer on, same speed as run B.
3. For each axis, write down:
   - twitch, from 1 (calm) to 5 (very twitchy)
   - the range of AOG's cross-track readout, for example ±3 cm
   - anything that only happens in one direction

**Pass:** both axes feel the same, and the twitch is better than before these changes.

Optional before/after: flash a build of commit eeba518 (before all of these changes), repeat runs B
and D, then flash the current build again.

## Test 8 (optional): is NAV-HPPOSLLH worth doing? (15 min, standing still)

This can also be done at home with the antennas outside.

NAV-PVT gives lat/lon on a 1.1 cm grid. Brief 5 would add NAV-HPPOSLLH for 0.1 mm. That only
helps if the position's epoch-to-epoch noise is smaller than the grid.

1. Plug the laptop into the **position (right) F9P's** USB port. It is usually `/dev/ttyACM0`.
2. Turn on NAV-HPPOSLLH on that USB port, every epoch, in RAM only:
   `ft hp-enable /dev/ttyACM0`
   (or use u-center: CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB = 1, RAM layer only). This doesn't touch the
   ESP32's UART.
3. With RTK fixed and the tractor parked: `ft capture /dev/ttyACM0 t8-hp --raw --minutes 5`
4. `ft hpposllh t8-hp_*.log`
5. Turn it off again: `ft hp-enable /dev/ttyACM0 --off`, or power-cycle the F9P.

**Result:**
- **DO:** noise under 5 mm. The grid shows as a staircase, so brief 5 is worth doing.
- **MAYBE:** 5–10 mm.
- **SKIP:** 10 mm or more. The grid is hidden in the noise.

## Afterwards

- "Debug raw" off. If you changed the USB baudrate, set it back to 115200.
- Keep the log folder, the downloaded config and this sheet together, so the logs can be looked at
  again later.

## Results sheet

Date: __________  Field: __________  Firmware build date: __________
AntDist setting: ______ cm  Antenna height: ______ cm  Right offset: ______ cm

| Test | Result | Numbers | Notes |
|---|---|---|---|
| 1 Antenna distance, parked | | deviation ____ cm | |
| 1 Antenna distance, driving | | deviation ____ to ____ cm; summary ____ % in band | |
| 2 RELPOSNED on time | | Stale ____, Dropped total ____, log ____ very poor epochs | |
| 3 Filter lag | | FilterPos=1 ____ % | |
| 4 Heading on 0° line | | N-S / E-W heading noise ____ ×, crossings ____ | |
| 5 Reversing | | slowest ____ epochs, heading moved ≤ ____ ° | |
| 6A Steering, logged | | lat noise N-S ____ / E-W ____ cm; lat sd N-S ____ / E-W ____ cm | |
| 6B Steering feel | | twitch N-S ____ / E-W ____ (1-5); XTE ±____ / ±____ cm | |
| 7 $PAOGI format | | broken in log ____ | |
| 8 HPPOSLLH (optional) | | noise N ____ / E ____ mm → DO / MAYBE / SKIP | |
