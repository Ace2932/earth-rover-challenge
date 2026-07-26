"""Watch the SDK/Chrome layer, because it fails silently.

The control chain is:

    policy -> HTTP -> SDK FastAPI -> headful Chrome (Agora) -> 4G -> bot

Chrome is in the critical path of every command. If that page wedges — a crashed
tab, a dropped Agora session, a laptop that slept — `/control` keeps returning
200 and nothing reaches the bot. From the client's side the failure is invisible.

The one thing that gives it away is `/data`'s own `timestamp`: a live page keeps
advancing it, a wedged one repeats the same value while still answering requests.

    python3 health.py --restart-cmd "docker compose restart sdk"

Run it next to the follower on the same box (see DEPLOYMENT.md).
"""
import argparse
import subprocess
import time


class StaleDetector:
    """Feed it each `/data` payload (or None for a failed request). Returns
    False once telemetry has stopped advancing for `stale_s`, and calls
    `restart` at most once per `cooldown_s`."""

    def __init__(self, stale_s=10.0, cooldown_s=60.0, clock=time.monotonic, restart=None):
        self.stale_s = stale_s
        self.cooldown_s = cooldown_s
        self.clock = clock
        self.restart = restart
        self.last_value = None      # last distinct telemetry timestamp
        self.last_change = None     # when we saw it change
        self.last_restart = None

    def _trigger(self):
        now = self.clock()
        if self.last_restart is not None and now - self.last_restart < self.cooldown_s:
            return
        self.last_restart = now
        if self.restart:
            self.restart()

    def observe(self, data):
        now = self.clock()
        if self.last_change is None:
            self.last_change = now

        if data is None:
            # A failed request is its own symptom: the SDK server is not answering.
            if now - self.last_change > self.stale_s:
                self._trigger()
                return False
            return True

        ts = data.get("timestamp") if isinstance(data, dict) else None
        if ts is None:
            # No timestamp to compare, but a response arrived at all — that is the
            # only liveness signal available, so take it.
            self.last_change = now
            return True

        if ts != self.last_value:
            self.last_value = ts
            self.last_change = now
            return True

        if now - self.last_change > self.stale_s:
            self._trigger()
            return False
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--base-url", default="http://localhost:8000")
    ap.add_argument("--stale", type=float, default=10.0,
                    help="seconds of unchanging telemetry before calling it wedged")
    ap.add_argument("--cooldown", type=float, default=120.0,
                    help="minimum seconds between restart attempts")
    ap.add_argument("--poll", type=float, default=2.0)
    ap.add_argument("--restart-cmd", default=None,
                    help="shell command to bring the SDK server back "
                         '(e.g. "docker compose restart sdk"); omit to only report')
    args = ap.parse_args()

    import requests

    def restart():
        if not args.restart_cmd:
            print("[health] telemetry is stale — no --restart-cmd, reporting only",
                  flush=True)
            return
        print(f"[health] telemetry is stale — running: {args.restart_cmd}", flush=True)
        try:
            subprocess.run(args.restart_cmd, shell=True, check=False, timeout=120)
        except subprocess.SubprocessError as e:
            print(f"[health] restart command failed: {e}", flush=True)

    det = StaleDetector(stale_s=args.stale, cooldown_s=args.cooldown, restart=restart)
    session = requests.Session()
    print(f"[health] watching {args.base_url}/data "
          f"(stale after {args.stale}s, cooldown {args.cooldown}s)", flush=True)
    healthy = True
    while True:
        try:
            data = session.get(f"{args.base_url}/data", timeout=3).json()
        except Exception:
            data = None
        now_healthy = det.observe(data)
        if now_healthy != healthy:
            print(f"[health] {'recovered' if now_healthy else 'UNHEALTHY'}", flush=True)
            healthy = now_healthy
        time.sleep(args.poll)


if __name__ == "__main__":
    main()
