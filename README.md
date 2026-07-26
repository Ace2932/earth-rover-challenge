# Earth Rover Challenge — entry (Aiden Fox)

Autonomous navigation entry for the **Earth Rover Challenge @ IROS 2026** (Pittsburgh,
Sept 30 – Oct 1, 2026). Standardized Earth Rover Mini+ fleet; all compute off-board —
you stream camera + GPS in, control commands out, over the Frodobots Remote Access SDK.

**Tracks:** Urban (GPS waypoints on sidewalks) · Indoor (image-goal) · Off-road (image-goal) ·
Marathon (~50 mi, all domains). This starter targets **Urban** first (GPS-waypoint follower),
which maps directly onto standard robot nav.

## What's here
| File | Role |
|---|---|
| `rover_client.py` | Resilient HTTP wrapper (session + retry/backoff) over the SDK's local server. |
| `geo.py` | Haversine distance + initial-bearing + angle-wrap (0=N, 90=E). |
| `waypoint_follower.py` | Bearing controller + GPS-course heading fusion, safety-stop, stuck detection, server-authoritative checkpoints, run logging, optional `--vision`. |
| `tests/` | `pytest` unit tests for geometry + control law + heading fusion. |
| `fake_sdk_server.py` | Stdlib fake SDK server — run the REAL HTTP client end-to-end, no bot. |
| `calibrate_heading.py` | Recover the bot's `orientation`→degrees mapping (run once per bot). |
| `CALL_DAY_RUNBOOK.md` | Exact live bring-up steps for the onboarding call. |
| `DEPLOYMENT.md` | Why this belongs on a cloud VM, and how to run it there. |
| `health.py` | Watches `/data`'s timestamp; restarts the SDK server when Chrome wedges. |
| `.env.example` | SDK + tuning config. |
| `vision/fetch_model.sh` | Download + verify the `--vision` checkpoint (not in the repo). |

## Setup
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt      # runtime + pytest
# only if you want --vision (torch, torchvision, pillow, opencv, ...):
.venv/bin/pip install -r vision/requirements.txt
```

## Quick start — works right now, no hardware
```bash
# A) pure sim (in-process kinematics)
.venv/bin/python waypoint_follower.py --mock

# B) full HTTP path against a fake SDK server (proves the live client/server integration)
.venv/bin/python fake_sdk_server.py 8777 &
SDK_BASE_URL=http://localhost:8777 .venv/bin/python waypoint_follower.py

# C) the same, under realistic 4G/urban conditions — noisy + biased GPS, laggy telemetry,
#    silently dropped control messages, and the server's real 15 m checkpoint tolerance
FAKE_GPS_SIGMA_M=1.5 FAKE_GPS_BIAS_M=8 FAKE_TELEMETRY_LATENCY_S=0.5 \
  FAKE_CONTROL_DROP_RATE=0.05 .venv/bin/python fake_sdk_server.py 8777 &
SDK_BASE_URL=http://localhost:8777 .venv/bin/python waypoint_follower.py
```
Run the tests: `PYTHONPATH=. .venv/bin/python -m pytest tests/ -q`

The fake server reproduces the SDK's real responses, including the `400` with
`proximate_distance_to_checkpoint` you get whenever you are not close enough — see
`.env.example` for every `FAKE_*` knob. All error injection is off by default.

### Built for a real 4G rover
- **Commands are streamed, not fired once (`commander.py`):** a background thread re-sends the
  current setpoint at `COMMAND_HZ` (20 Hz, matching the SDK's own reference teleop), because
  `/control` is an unacked Agora RTM message — an HTTP 200 means the browser called
  `sendMessage`, not that the bot heard it. `set()` never blocks on the network.
- **Stale setpoint decays to a stop:** if the control loop stops publishing for
  `SETPOINT_STALE_S` (0.5 s) — blocked on a slow request, wedged, anything — the stream falls
  to `(0,0)` instead of holding throttle. An in-process watchdog, for free.
- **Out-of-process watchdog (`watchdog.py`, `--watchdog`):** `try/finally` cannot survive
  `kill -9`, an OOM, or the laptop sleeping, and the rover latches its last command. A second
  process watches a heartbeat and stops the rover if it goes stale. Verified: `kill -9` on the
  follower produced a stop **1.07 s** later and zero false stops while healthy. Whether the bot's
  own firmware zeroes on link loss is still unconfirmed — see the runbook's first-session check.
- **Hardened stop:** on exit the stop is retried, and the telemetry is read back to confirm
  `speed` actually fell to zero — an HTTP 200 is not evidence the rover stopped.
- **Request resilience:** `RoverClient` retries with backoff, and the control loop tolerates
  whatever gets through: a failed step is skipped rather than fatal, and the run gives up only
  after `MAX_CONSECUTIVE_ERRORS` in a row. Measured against the fake server — **30% injected
  fault rate: completes 3/3 waypoints; 60%: degrades to 1/3 within the time limit but never
  crashes and never leaves the rover driving.** The exit stop is retried and never raises, so
  it cannot mask the original failure or strand a moving rover.
- **Heading estimation (`heading.py`):** a complementary filter. Yaw is dead-reckoned
  between fixes (gyro if trusted, else the commanded angular); corrections come only from a
  GPS course measured over an **odometry** baseline of `HEADING_MIN_MOVE_M`, rejected if the
  wheels moved but the GPS did not. A chord measured across a turn is the AVERAGE heading
  over the window, so it is de-biased by half the turn rather than discarded — discarding it
  meant no correction ever landed above ~3.4 deg/s of turn, and the rover orbited checkpoints
  (#44). The magnetometer
  seeds the filter once and is never read again. Under sigma=1.5 m GPS noise this holds
  ~2 deg median heading error; taking the course over a short baseline (the previous design)
  gave ~88 deg and preferred it over the magnetometer 93% of the time.
- **Server-authoritative checkpoints:** only advances when `/checkpoint-reached` returns 200 —
  and starts asking at `CHECKPOINT_RADIUS_M` (20 m), because the acceptance tolerance is the
  server's (15 m), not ours. Its refusal carries `proximate_distance_to_checkpoint`, which the
  loop then navigates and measures progress on, and logs as `sdist` — except while detouring,
  when that distance is to the checkpoint rather than to where we are driving.
- **Stuck recovery (`recovery.py`):** no progress for `STUCK_S` no longer ends the run. The
  ladder is: back up and turn (`RECOVERY_TRIES` attempts, alternating direction) → approach the
  checkpoint from `RECOVERY_OFFSET_M` to the side → only then record an intervention via
  `/interventions/start` and stop. The detour point is deliberately **not** a checkpoint, so it
  is never claimed as one.
- **Fails loud, resumes correctly:** a refused `/start-mission` (400 "Bot unavailable for SDK")
  or an empty checkpoint list aborts with `MissionUnavailable` instead of reporting
  `COMPLETE — 0/0`. Checkpoints are ordered by `sequence`, and a restart resumes from the
  server's `latest_scanned_checkpoint` rather than driving the whole route again.
- **Run logging:** `--log run.csv` records pose/heading-source/cmd every step for tuning.

Both quick-start commands end in `COMPLETE — 3/3 waypoints`. (B) exercises the real `requests` client, JSON
shapes, and the orientation→heading pipeline — verified against the SDK's actual `main.py`.

## Live (after registration + a bot/allocation)
1. Activate a bot (or claim challenge testing allocation) → SDK token at
   https://my.frodobots.com/owner/settings.
2. Clone the SDK, set env, run its server:
   ```bash
   git clone https://github.com/frodobots-org/earth-rovers-sdk
   cd earth-rovers-sdk && pip3 install -r requirements.txt
   export SDK_API_TOKEN=... BOT_SLUG=... MISSION_SLUG=mission-1
   hypercorn main:app --reload        # serves http://localhost:8000
   ```
3. In this repo, run the follower against it:
   ```bash
   cp .env.example .env      # (optional; or export vars inline)
   python3 waypoint_follower.py                 # waypoints from /checkpoints-list
   # or:  python3 waypoint_follower.py --route my_route.json
   ```

## ⚠️ Calibrate heading before trusting live steering
The SDK `orientation` field's units aren't documented as degrees. Once: drive straight for
~10 m, log GPS-track bearing vs reported `orientation`, and set `HEADING_SCALE/OFFSET/SIGN`
in `.env` so `waypoint_follower` heading matches true compass heading (0=N, 90=E). The mock
backend is already self-consistent, so tune the *control gains* (`KP_ANG`, `CRUISE`,
`ALIGN_DEG`) in sim, then calibrate *heading* on the bot.

## Roadmap (post-baseline)
- **Perception for Urban → `vision/` (built, see `vision/README.md`):** behavior-cloning
  sidewalk-keeping policy fused with the GPS follower. Pipeline proven on a synthetic task
  (val_mse ~1e-4); swap in real FrodoBots data (needs HF access) to make it competitive.
- **Recovery:** intervention API + stuck-detection (no GPS progress → back off / re-plan).
- **Off-road / Indoor:** image-goal policy (no GPS) — bigger lift, reuse the control loop.
- Log frames to `frames/` for offline eval; add a small dashboard.

## Links
- Challenge: https://earth-rover-challenge.github.io/  · register: https://forms.gle/S4qWszRZpeNaDuZHA
- SDK: https://github.com/frodobots-org/earth-rovers-sdk
- Datasets: FrodoBots-2K, Berkeley-FrodoBots-7K (HuggingFace)
- Contact: michael.cho@frodobots.com
