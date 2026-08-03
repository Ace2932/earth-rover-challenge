# Call-day runbook — Earth Rover onboarding (Tue 2026-07-07, 6:00–6:30 PM)

Team **Fennec**. Everything below is already built + tested against a fake SDK server;
this is the live bring-up once you have bot access.

## On the call — ask these 3 (they unlock the resources)
1. How are the **20 hrs/week testing allocations** claimed/booked, and when do they start?
2. **SDK token + bot access** — how/when do I get a `SDK_API_TOKEN` and `BOT_SLUG`?
3. Can a **solo team** enter the **Marathon** track, or Urban/Off-road only?
(also worth: which Urban city sites are closest / available; what `MISSION_SLUG`s exist.)

## The moment you have a token → go live (≈10 min)
```bash
# 1. real SDK server (its own venv; needs Chrome 143+)
git clone https://github.com/frodobots-org/earth-rovers-sdk && cd earth-rovers-sdk
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
export SDK_API_TOKEN=...  BOT_SLUG=...  MISSION_SLUG=mission-1
export CHROME_EXECUTABLE_PATH="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
.venv/bin/hypercorn main:app --reload        # serves http://localhost:8000

# 2. sanity: real GPS coming back?
curl -s http://localhost:8000/data | python3 -m json.tool

# 3. calibrate heading ONCE (clear ~5 m ahead)
cd ~/codebases/earth-rover-challenge
.venv/bin/python calibrate_heading.py         # prints HEADING_SCALE/OFFSET/SIGN — export them

# 4. drive the GPS-waypoint baseline
export HEADING_SCALE=... HEADING_OFFSET=... HEADING_SIGN=...
.venv/bin/python waypoint_follower.py         # waypoints from /checkpoints-list
```

## Driving YOUR OWN bot (no mission) — the offline route path
Your Mini+ has no `MISSION_SLUG`, and without one the SDK's mission API is unusable:
`/checkpoints-list` returns `{}` and `/checkpoint-reached` answers `500`. So the default
invocation cannot run at all on it. Use a route you record yourself.

```bash
# 0. SDK server on 8001 (Motif's uvicorn owns 8000 on this laptop)
export SDK_BASE_URL=http://localhost:8001

# 1. FIRST outdoor telemetry check — before anything drives
curl -s $SDK_BASE_URL/data | python3 -m json.tool | head -20
#    latitude/longitude of 1000 and fix_quality 0 mean NO LOCK. The follower now refuses
#    to drive on that (#77) rather than steering on a 13 000 km phantom bearing — but you
#    still want a lock before you start. Note gps_signal here vs indoors: that measurement
#    is one of the three open unknowns.

# 2. calibrate heading ONCE, ~5 m clear ahead
.venv/bin/python calibrate_heading.py        # export the HEADING_SCALE/OFFSET/SIGN it prints

# 3. teleop the route once while recording (this tool only READS, it cannot drive)
.venv/bin/python capture_route.py park_lap.json      # Ctrl-C when the lap is done

# 4. drive it autonomously, governor OFF and slow
GPS_SIGNAL_GOOD=0 GPS_SIGNAL_POOR=0 CRUISE=0.3 \
  .venv/bin/python waypoint_follower.py --route park_lap.json --watchdog --log run1.csv
```

`GPS_SIGNAL_GOOD=0 GPS_SIGNAL_POOR=0` disables the speed governor. Do this on run 1: the
units of `gps_signal` are still unmeasured, and the bench read **16 with zero GPS fix**, so
the shipped thresholds are a guess that could crawl the whole run.

### If it refuses to drive outdoors with a lock you can see

```
[follower] WARNING: GPS reports no fix (fix_quality 0)
[follower] no usable position fix (no GPS lock) — stopping
```

`fix_quality` is **undocumented** and has been observed exactly once — indoors, alongside the
lat/lon 1000 sentinel, so the two were perfectly correlated and the sentinel alone explains
what was seen. It has **never** been observed on a bot with a real lock. "0 = invalid" is the
NMEA reading, not a measurement of this bot.

If the coordinates look sane and only this fires, the assumption is wrong for the Mini+:

```bash
IGNORE_FIX_QUALITY=1 ... python waypoint_follower.py --route park_lap.json --watchdog
```

Then **write down what `fix_quality` actually reads outdoors with a lock** — that turns a
guess into a measurement and the flag can go away. The coordinate check (`|lat| > 90`) is not
affected by the flag and cannot be switched off; it needs no undocumented field.

### It gives up after ~4 s without a lock
`MAX_CONSECUTIVE_ERRORS` (20) at `LOOP_HZ` 5 is about four seconds. A lock that comes back
resets the counter, so bridges and awnings are survivable — but a long urban canyon is not.
Raise it if a run dies somewhere you expected to lose signal.

Arrival on a route file is **local** — within `LOCAL_ARRIVE_M` (5 m) of our own fix, and no
checkpoint is claimed. It is the only thing that works without a mission, and it proves less
than a mission run does: the judge of arrival is the same fix you are steering on.

## ⚠️ First live session: find out whether the bot has its own watchdog
Send a command, kill the client hard, and watch:
```bash
curl -s -XPOST localhost:8000/control -H 'content-type: application/json' \
  -d '{"command":{"linear":0.3,"angular":0,"lamp":0}}'   # keep ~2 m clear ahead
# ...then stop sending. Does it coast to a stop by itself, and how fast?
```
Ask Frodobots directly whether the firmware zeroes on RTM loss and after what timeout.
Until that is answered, drive with `--watchdog`:
```bash
.venv/bin/python waypoint_follower.py --watchdog --log run1.csv
```

## Sign check (the one thing calibration can't fully resolve in one run)
Start the follower. If it steers *away* from the target (error grows, spins):
`export HEADING_SIGN=-1` and negate `HEADING_OFFSET`, re-run. That's the only likely gotcha.

## Rehearse anytime before the call (no bot needed)
```bash
cd ~/codebases/earth-rover-challenge
.venv/bin/python fake_sdk_server.py 8777 &
SDK_BASE_URL=http://localhost:8777 .venv/bin/python waypoint_follower.py   # 3/3 over HTTP
```

## ⛔ Do not run /end-mission
The SDK's docs: *"should only be used in case of emergency. If you run this endpoint you will
lose all your progress."* Nothing in this repo calls it, and the client method is named
`emergency_abort_lose_all_progress(confirm=True)` so it cannot be reached by accident.
To stop safely, just stop the rover — the mission stays open and a restart resumes it.

## Troubleshooting
- `Bot unavailable for SDK` → bot not assigned / another session holds it; check allocation.
  The follower now aborts with `cannot start: /start-mission refused: ...` (exit 2) instead
  of pretending the run completed.
- Follower crashed mid-mission → just restart it. It reads `latest_scanned_checkpoint` and
  picks up at the next checkpoint (`resuming: server reports N/M already reached`).
- No GPS / `orientation` weird → confirm bot is outdoors with signal; re-run calibration.
- **Check what `gps_signal` actually means** on the first live run: `curl -s localhost:8000/data`
  outdoors vs. beside a building. The guards assume higher = better on a 0..31-ish scale. If it
  turns out to be the opposite (an HDOP-style figure), set `GPS_SIGNAL_GOOD=0 GPS_SIGNAL_POOR=0`
  to disable the scaling until the mapping is confirmed.
- `ABORT: battery ...` → the run stopped on purpose. Charge, then restart; it resumes.
- Chrome errors → fix `CHROME_EXECUTABLE_PATH`; SDK needs Chrome 143+.
- Follower turns in place forever → heading sign/offset wrong → redo calibration + sign check.
- Overshoots checkpoints → lower `CRUISE`, raise `KP_ANG`, or widen `CHECKPOINT_RADIUS_M`.
- Rover wedged on a curb → it now backs up and turns by itself (up to `RECOVERY_TRIES`), then
  tries approaching from `RECOVERY_OFFSET_M` to the side. Watch for
  `recording an intervention and stopping` — that is the point a human is needed.
- **Keep clearance behind the rover.** Recovery reverses at `RECOVERY_REVERSE_THROTTLE` for
  `RECOVERY_REVERSE_S` with no rear obstacle sensing (there is a rear camera; nothing reads it
  yet). Set `RECOVERY_REVERSE_S=0` to disable reversing if the site is tight.

## After the baseline drives
Camera sidewalk-keeping (Urban) trained on FrodoBots-2K / Berkeley-FrodoBots-7K — the piece
that turns "reaches GPS points" into "stays on the sidewalk between them." See README roadmap.
