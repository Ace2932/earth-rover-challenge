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
| `rover_client.py` | Thin HTTP wrapper over the SDK's local server (`/data`, `/v2/front`, `/control`, Missions API). |
| `geo.py` | Haversine distance + initial-bearing + angle-wrap (0=N, 90=E). |
| `waypoint_follower.py` | Proportional bearing controller. `--mock` sim (no hardware) or live. |
| `fake_sdk_server.py` | Stdlib fake SDK server — run the REAL HTTP client end-to-end, no bot. |
| `calibrate_heading.py` | Recover the bot's `orientation`→degrees mapping (run once per bot). |
| `CALL_DAY_RUNBOOK.md` | Exact live bring-up steps for the onboarding call. |
| `.env.example` | SDK + tuning config. |

## Setup
```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

## Quick start — works right now, no hardware
```bash
# A) pure sim (in-process kinematics)
.venv/bin/python waypoint_follower.py --mock

# B) full HTTP path against a fake SDK server (proves the live client/server integration)
.venv/bin/python fake_sdk_server.py 8777 &
SDK_BASE_URL=http://localhost:8777 .venv/bin/python waypoint_follower.py
```
Both end in `COMPLETE — 3/3 waypoints`. (B) exercises the real `requests` client, JSON
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
