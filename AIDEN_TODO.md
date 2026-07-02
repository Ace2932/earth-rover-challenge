# Fennec × Earth Rover Challenge — what YOU need to do

Ordered by clock. Code side is done + tested; these are the things that need your
accounts, hardware, or a human at the keyboard.

## ✅ Dataset access — DONE (Berkeley-7K gate accepted + HF login confirmed)
The real data (**`BitRobot/Berkeley-FrodoBots-7K`**) turned out to be a **Zarr store split
across 24 tar.gz parts, ~769 GB** — reannotated MBRA nav labels. Not streamable as tidy rows;
it must be downloaded + reconstructed. **So real-data training is a cloud/big-disk job, not a
laptop one.** The synthetic-trained policy is your working baseline until then.

## 🔴 Real-data training (when you want the competitive vision policy) — needs a cloud box
On a rented GPU box with ~1 TB disk:
1. `bash vision/download_berkeley.sh ./berkeley7k`  (24 parts → cat → extract zarr)
2. `python3 vision/inspect_zarr.py ./berkeley7k/.../dataset_cache.zarr`  (prints real shapes)
3. Send me the shapes → I write the exact Dataset + retrain resnet18 on `action_mbra`.
> Cheaper alt: FrodoBots-2K raw rides (public, MP4/CSV) for a smaller pretrain — `vision/README.md`.

## 🟠 Tue 2026-07-07, 6:00 PM — onboarding call
Ask the 3 questions (in `CALL_DAY_RUNBOOK.md`): testing-allocation booking, SDK token/bot
access, solo Marathon eligibility. **Book the 30-min slot** on the Calendly redirect if not done.

## 🟠 When you get a bot token (call or after) — go live (~10 min)
Follow `CALL_DAY_RUNBOOK.md`:
1. clone the SDK, its own venv, set `SDK_API_TOKEN`/`BOT_SLUG`/`MISSION_SLUG`, run `hypercorn main:app`.
2. `curl localhost:8000/data` — real GPS?
3. `.venv/bin/python calibrate_heading.py` — but note: heading now **auto-fuses from GPS
   course while moving**, so calibration is a nice-to-have, not a blocker.
4. `.venv/bin/python waypoint_follower.py --log run1.csv` — drive the baseline. Watch the
   sign-check (if it steers away from target, flip `HEADING_SIGN`).

## 🟡 Decisions only you can make
- **Buy an Earth Rover Mini+?** (~practice hardware) vs. rely on the challenge testing
  allocation. Ask on the call which is enough.
- **Pittsburgh travel** Sept 27–Oct 1 (confirm on the call whether solo attendance is expected).
- **Cloud GPU** for full vision training (Berkeley-7K is 769 GB) — a rented A10/A100 for a few
  hours once the loader's wired; MPS on your Mac handles a subset.

## 🟢 Optional / build-in-public
- The `--log run.csv` output is ready-made content (@outofofficefox) — plot pose + heading source.
- Public repo: this is `~/codebases/earth-rover-challenge` (local git only). Push to
  `github.com/Ace2932/...` when you want it visible (say the word, I'll set it up).

## Done for you (no action needed)
Team registered (Fennec) · GPS-waypoint follower (safety-stop, retry, heading fusion, stuck
detection, server-auth checkpoints, logging) · fake-server integration + 11 unit tests + 60%
fault-rate resilience all passing · vision sidewalk-keeping pipeline proven on synthetic ·
call-day runbook · heading calibration tool.
