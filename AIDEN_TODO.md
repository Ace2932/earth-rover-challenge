# Fennec × Earth Rover Challenge — what YOU need to do

Ordered by clock. Code side is done + tested; these are the things that need your
accounts, hardware, or a human at the keyboard.

## 🔴 Now — unlock the real dataset (2 browser steps + 1 terminal)
The gated dataset is **`BitRobot/Berkeley-FrodoBots-7K`** (webdataset, 769 GB, reannotated
MBRA labels). I opened its page in Chrome.
1. **Sign in** to Hugging Face (top-right of that tab).
2. Click **"Agree and access repository"** (shares your email/username with the authors —
   that's why it has to be you).
3. Create a **read token** at https://huggingface.co/settings/tokens, then in a terminal:
   ```bash
   cd ~/codebases/earth-rover-challenge
   ! .venv/bin/huggingface-cli login       # paste the token when prompted
   ```
   (the `!` runs it in this Claude session so I can verify access right after.)
Then tell me — I'll stream a real sample, confirm the `action_mbra` shape, and wire the
real data loader + retrain the sidewalk policy on actual driving data.

> Alt if you'd rather not gate: FrodoBots-2K raw is public but needs manual ride downloads +
> MP4/CSV parsing (see `vision/README.md`). Berkeley-7K is the better path.

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
