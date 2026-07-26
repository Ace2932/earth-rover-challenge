"""Out-of-process runaway protection for the rover.

The follower's `try/finally` stops the rover on an exception or Ctrl-C. It cannot
stop it after `kill -9`, an OOM kill, a wedged interpreter, or the laptop
suspending — and the rover latches its last command: `/control` is a
fire-and-forget Agora RTM message, and no bot-side watchdog is documented
anywhere in the SDK.

So run this alongside. The Commander refreshes a heartbeat file every time a
command actually goes out; if that file stops being refreshed, this process
starts sending stop commands and keeps sending them. If the file is *removed*,
the follower exited cleanly and this exits too.

    python3 watchdog.py --heartbeat /tmp/erc.hb

VERIFY FIRST, on the first live session: send linear=0.3, `kill -9` the client,
and watch whether the rover coasts to a stop on its own. If the firmware already
zeroes on RTM loss this is belt-and-braces; if it does not, this is the only
thing standing between a crashed laptop and a rover in traffic. Ask Frodobots
directly — the answer belongs in the README either way.
"""
import argparse
import os
import time


class Watchdog:
    """One `tick()` per poll interval. Returns False when it is time to exit."""

    def __init__(self, stop, heartbeat_mtime, clock=time.time, timeout_s=1.0):
        self.stop = stop
        self.heartbeat_mtime = heartbeat_mtime
        self.clock = clock
        self.timeout_s = timeout_s
        self.seen = False          # has the heartbeat ever existed?
        self.stops = 0

    def tick(self):
        mtime = self.heartbeat_mtime()
        if mtime is None:
            if not self.seen:
                return True        # follower has not started yet; wait
            return False           # it removed the file: clean shutdown
        self.seen = True
        if self.clock() - mtime > self.timeout_s:
            self.stop()
            self.stops += 1
        return True


def _mtime_reader(path):
    def read():
        try:
            return os.stat(path).st_mtime
        except OSError:
            return None
    return read


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--heartbeat", required=True, help="path the Commander refreshes")
    ap.add_argument("--base-url", default=os.getenv("SDK_BASE_URL", "http://localhost:8000"))
    ap.add_argument("--timeout", type=float, default=1.0,
                    help="seconds without a heartbeat before stopping the rover")
    ap.add_argument("--poll-hz", type=float, default=5.0)
    args = ap.parse_args()

    from rover_client import RoverClient
    client = RoverClient(base_url=args.base_url, timeout=1.0, retries=1)

    def stop():
        try:
            client.control(0.0, 0.0)
            print(f"[watchdog] heartbeat stale — sent stop", flush=True)
        except Exception as e:
            print(f"[watchdog] stop FAILED: {e}", flush=True)

    w = Watchdog(stop=stop, heartbeat_mtime=_mtime_reader(args.heartbeat),
                 timeout_s=args.timeout)
    print(f"[watchdog] watching {args.heartbeat} (timeout {args.timeout}s)", flush=True)
    period = 1.0 / max(1e-6, args.poll_hz)
    while w.tick():
        time.sleep(period)
    print("[watchdog] heartbeat removed — follower exited cleanly, stopping", flush=True)


if __name__ == "__main__":
    main()
