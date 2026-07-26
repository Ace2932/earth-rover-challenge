# Fennec × Earth Rover Challenge — what YOU need to do

Ordered by what blocks what. The code has moved a long way; the things below need your
accounts, the hardware, or an answer from Frodobots.

Last reviewed: 2026-07-26.

## 🔴 Three questions only a real bot can answer

These are the highest-value hour you can spend, and every one of them is currently a
**documented guess** in the code. All three are cheap to check on the first live session.

1. **Does the firmware stop by itself when the link drops?**
   `/control` is an unacked Agora RTM message. If the bot latches its last command, a
   crashed laptop leaves a rover driving. `watchdog.py` covers process death, but nothing
   covers a dead machine. Procedure is in `CALL_DAY_RUNBOOK.md`: command `linear=0.3`,
   stop sending, watch. Then ask Frodobots directly and put the answer in the README.
2. **What are `gps_signal`'s units?** The telemetry guard assumes higher = better on a
   0..31-ish scale and scales speed down between `GPS_SIGNAL_POOR` and `GPS_SIGNAL_GOOD`.
   If it is an HDOP-style figure (lower = better) the scaling is backwards and the rover
   crawls the whole mission — measured at **3× slower** in rehearsal. `curl /data` outdoors
   versus beside a building settles it. `GPS_SIGNAL_GOOD=0 GPS_SIGNAL_POOR=0` disables it.
3. **Is `latest_scanned_checkpoint` a count or an index?** The follower resumes from it
   after a crash. If it is 1-based, resume starts one checkpoint late. One character to fix,
   30 seconds to check.

## 🔴 Calibrate before the first real run

- `calibrate_heading.py` — recovers the magnetometer → degrees mapping. Since the heading
  filter only **seeds** from the magnetometer, a bad calibration costs one wrong turn rather
  than the run, but it is still worth doing. Keep ~5 m clear ahead; the script stops the
  rover in a `finally` now, but never `kill -9` it.
- `YAW_RATE_DPS` (default 90 °/s at `angular=1.0`) is the dead-reckoning model. Drive a
  known 90° turn and time it. Getting this wrong costs distance and time on every leg —
  measured at **2× the odometry** for the same route when yaw was 20% off.
- Then consider `USE_GYRO=1`. The code path exists and is off by default because the SDK
  documents neither the axis order nor the units of `/data`'s `gyros`. Confirm both and it
  becomes the cheapest accuracy upgrade available.

## 🟠 Deploy somewhere that does not sleep

`DEPLOYMENT.md` — run the SDK server, the follower and `health.py` on a small cloud VM in the
bot's region. The challenge states off-board compute is unlimited, so there is no reason for
your laptop's Wi-Fi and sleep settings to be in the critical path of every command. Ask on the
call where the bot you are allocated actually lives, since a wrong region adds latency to an
already ~500 ms video path.

## 🟠 Vision: the shipped policy is a weak prior, not a competitor

Read `vision/MODEL_CARD.md` before trusting it. Trained on **three rides of one city**, with a
64% left-turn bias, no held-out real-data evaluation, no obstacle awareness and no uncertainty
output. The follower gates it hard by default and it is off unless you pass `--vision`.

- The checkpoint is not in the repo (43 MB). `bash vision/fetch_model.sh` fetches and verifies
  it — **but the release asset does not exist yet**; the script prints the one `gh release
  create` command when you decide to publish it. That is your call, not mine: it publishes a
  FrodoBots-derived artifact to a public repo.
- The competitive version needs the full dataset on a cloud GPU. `vision/colab_frodobots.ipynb`
  is the smaller path; Berkeley-7K (769 GB, zarr in 24 tar parts) is the bigger one.
- The obstacle-stop head is **plumbed and honestly negative**: trained on the getting-started
  subset it learned to say "never blocked" for five epochs, because positives are 2% of the
  data. Numbers are in `vision/README.md`. Do not enable it on a rover on that evidence.

## 🟡 Decisions only you can make

- **Buy an Earth Rover Mini+?** versus relying on the challenge testing allocation. Worth
  asking which is enough, now that there is a stack worth testing.
- **Pittsburgh travel**, Sept 27 – Oct 1.
- **Cloud GPU** for the real vision training.
- **Publish the model checkpoint** to a GitHub Release, or keep it local.

## 🟢 State of the code

Reviewed and rebuilt across ~30 issues. What exists now, all with tests:

| Area | What it does |
|---|---|
| `heading.py` | Complementary filter: yaw dead-reckoned between fixes, corrected only by a GPS course over an odometry baseline, de-biased across turns, gated on net rotation. ~2° median error at σ=1.5 m, against ~88° for the original design. |
| `commander.py` | Streams the setpoint at 20 Hz; a stale setpoint decays to a stop. |
| `watchdog.py` | Separate process; stops the rover if the follower dies. Measured 1.07 s after `kill -9`. |
| `recovery.py` | Back up, turn, re-approach from the side, then record an intervention and stop. |
| `telemetry.py` | Battery floor, GPS-quality speed cap, commanded-vs-actual motion check. |
| `blocked.py` | Obstacle-stop decision with hysteresis. Inert until a policy carries the head. |
| `health.py` | Watches `/data`'s timestamp; restarts the SDK server when Chrome wedges silently. |
| `fake_sdk_server.py` | Reproduces the real 400s, GPS noise and bias, telemetry latency, dropped commands, battery drain, and now imperfect yaw. |
| CI | Unit suite plus a real end-to-end drive, including under 30% injected faults. |

Dress rehearsal against the fake server, including an imperfect rover: completes 3/3 under
σ=3 m GPS noise, 12 m bias, 0.8 s latency, 15% dropped commands, 10% 503s and 20% yaw error;
parks on a flat battery; and refuses to start — loudly, exit 2 — when the bot is unavailable.

### What the harness still cannot tell you

Worth knowing before you read "completes 3/3" as reassurance:

- **The linear model is exact.** Commanded throttle maps to speed with no slip, no gradient,
  no battery sag. The heading filter's odometry baseline (`HEADING_MIN_MOVE_M`) is measured
  from commanded speed when telemetry has none, so it is trusting a number the sim guarantees
  and a real rover does not.
- **The rover is a point.** No width, no turning radius, no wheels to catch a curb — so the
  recovery ladder is only ever exercised against a rover that is stuck by fiat, never one that
  is stuck in a way backing up would actually fix.
- **The world is empty.** No obstacles, no pedestrians, no kerbs, no sidewalk edges. Nothing in
  the harness can exercise the obstacle-stop path, which is part of why its training is a
  negative result rather than a weak positive.
- **GPS error is zero-mean noise plus a rotating bias.** Real urban GPS has multipath: sudden
  jumps of tens of metres near buildings, correlated with exactly the places the course goes.
- **Latency is constant.** Real 4G latency is spiky, and a spike is what turns a stale frame
  into a wrong decision.

None of that makes the rehearsal worthless — it caught two real bugs. It means "it completed
in sim" is evidence about the code, not about the rover.
