# Deployment — run this on a cloud VM, not your laptop

## The chain you are actually depending on

```
policy process (this repo)
  │  HTTP
  ▼
SDK FastAPI  (hypercorn main:app, port 8000)
  │  CDP
  ▼
headful Chrome 143+  (Agora SDK)
  ├── RTM  (text)  ──4G──►  bot firmware     ← control commands + telemetry
  └── RTC  (video) ──4G──►  front/rear cams  (~20 Hz, ~500 ms)
```

Four things follow from that picture, and they are the whole argument for this document:

1. **Chrome is in the critical path of every command.** If the page wedges, `/control` keeps
   returning 200 and nothing reaches the bot. Silent from the client's side — hence
   `health.py`, which watches `/data`'s own `timestamp` for the one observable difference
   between a live page and a wedged one.
2. **Commands are unacked** (Agora RTM `sendMessageToPeer`). HTTP 200 means the browser called
   `sendMessage`. See `commander.py` / `watchdog.py`.
3. **Only one SDK session may hold a bot** — "bots controlled by other players are not
   available". There is no hot standby. Your single process is the single point of failure, so
   it has to be crash-restartable *mid-mission* (it is: the follower resumes from
   `latest_scanned_checkpoint`).
4. **Compute is off-board and unlimited** per the challenge rules. So there is no reason to
   leave your laptop's Wi-Fi, sleep settings, and desktop Chrome in the failure set.

## Recommended layout

One small VM **in the bot's region**, running three processes:

| Process | Job | Restart policy |
|---|---|---|
| SDK server | Chrome + Agora + the HTTP surface | always |
| `waypoint_follower.py --watchdog` | the actual driving | on-failure (resumes mid-mission) |
| `health.py --restart-cmd ...` | restarts the SDK server when telemetry stops advancing | always |

You SSH in; nothing depends on your local machine staying awake.

```bash
# on the VM
git clone https://github.com/frodobots-org/earth-rovers-sdk
git clone https://github.com/Ace2932/earth-rover-challenge
cd earth-rover-challenge
cp .env.example .env      # fill in SDK_API_TOKEN, BOT_SLUG, MISSION_SLUG
docker compose up -d
docker compose logs -f follower
```

`docker-compose.yml` here builds the follower and the health watcher and expects the SDK
server reachable at `SDK_BASE_URL`. The SDK ships its own `Dockerfile` and
`docker-compose.yml` — run those alongside rather than vendoring them, so SDK updates are a
`git pull` in that repo.

### Without Docker

`deploy/` has systemd units. `systemctl enable --now erc-follower erc-health`.

## Sizing

Modest: the follower is a 5 Hz control loop and a 20 Hz command streamer. Chrome decoding two
video streams is the real cost — 2 vCPU / 4 GB is comfortable. Add a GPU only if you run
`--vision` with a large backbone; resnet18 at 96 px is fine on CPU.

## What still is not covered

- **A VM that dies outright** takes the watchdog with it. The rover keeps its last command
  unless the firmware has its own timeout — still unconfirmed, see the runbook's first-session
  check. This is the strongest argument for asking Frodobots directly.
- **Region choice is a guess** until you know where the bot you are allocated actually is. Ask
  on the call; a wrong region adds latency to an already ~500 ms video path.
- Nothing here makes the single-session limit go away.
