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
| `.env.example` | SDK + tuning config. |

## Quick start — sim (no hardware, works right now)
```bash
pip3 install -r requirements.txt
python3 waypoint_follower.py --mock      # drives a synthetic 3-waypoint route, prints convergence
```
Expected: each waypoint reported "reached", ending `COMPLETE — 3/3 waypoints`.

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
- **Perception for Urban:** stay-on-sidewalk / obstacle-stop from `front_frame` (the
  FrodoBots-2K + Berkeley-FrodoBots-7K datasets are labeled driving data — train or
  fine-tune a lane/keepout policy).
- **Recovery:** intervention API + stuck-detection (no GPS progress → back off / re-plan).
- **Off-road / Indoor:** image-goal policy (no GPS) — bigger lift, reuse the control loop.
- Log frames to `frames/` for offline eval; add a small dashboard.

## Links
- Challenge: https://earth-rover-challenge.github.io/  · register: https://forms.gle/S4qWszRZpeNaDuZHA
- SDK: https://github.com/frodobots-org/earth-rovers-sdk
- Datasets: FrodoBots-2K, Berkeley-FrodoBots-7K (HuggingFace)
- Contact: michael.cho@frodobots.com
