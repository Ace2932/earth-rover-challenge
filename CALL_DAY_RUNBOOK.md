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

## Sign check (the one thing calibration can't fully resolve in one run)
Start the follower. If it steers *away* from the target (error grows, spins):
`export HEADING_SIGN=-1` and negate `HEADING_OFFSET`, re-run. That's the only likely gotcha.

## Rehearse anytime before the call (no bot needed)
```bash
cd ~/codebases/earth-rover-challenge
.venv/bin/python fake_sdk_server.py 8777 &
SDK_BASE_URL=http://localhost:8777 .venv/bin/python waypoint_follower.py   # 3/3 over HTTP
```

## Troubleshooting
- `Bot unavailable for SDK` → bot not assigned / another session holds it; check allocation.
- No GPS / `orientation` weird → confirm bot is outdoors with signal; re-run calibration.
- Chrome errors → fix `CHROME_EXECUTABLE_PATH`; SDK needs Chrome 143+.
- Follower turns in place forever → heading sign/offset wrong → redo calibration + sign check.
- Overshoots checkpoints → lower `CRUISE`, raise `KP_ANG`, or widen `CHECKPOINT_RADIUS_M`.

## After the baseline drives
Camera sidewalk-keeping (Urban) trained on FrodoBots-2K / Berkeley-FrodoBots-7K — the piece
that turns "reaches GPS points" into "stays on the sidewalk between them." See README roadmap.
