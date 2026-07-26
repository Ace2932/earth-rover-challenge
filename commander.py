"""Streams the rover's current setpoint on a background thread.

Why not just call `/control` from the control loop:

  * Commands reach the bot over Agora RTM, unacked. `POST /control` returning 200
    means the SDK's browser called `sendMessage`, not that the bot heard it. The
    only defence against a lost message is to keep sending.
  * The reference teleop in the SDK's own examples streams at 20 Hz. A loop that
    sends once per iteration, inline, behind a 5 s timeout and 3 retries can stall
    for >15 s on a bad 4G moment — with the previous command still latched.
  * A setpoint nobody has refreshed is not a command, it is a symptom. If the
    controller stops updating, the stream decays to zero rather than holding
    throttle: an in-process watchdog for free.

`watchdog.py` covers what this cannot — the process dying outright.
"""
import os
import threading
import time


class Commander:
    def __init__(self, send, hz=20.0, stale_s=0.5, heartbeat_path=None,
                 stop_attempts=10, stop_gap_s=0.05, clock=time.time):
        """send(linear, angular): may block or raise; the thread absorbs both."""
        self.send = send
        self.period = 1.0 / max(1e-6, hz)
        self.stale_s = stale_s
        self.heartbeat_path = heartbeat_path
        self.stop_attempts = stop_attempts
        self.stop_gap_s = stop_gap_s
        self.clock = clock

        self._lock = threading.Lock()
        self._setpoint = (0.0, 0.0)
        self._setpoint_t = clock()
        self._closed = False
        self.failures = 0
        self.sent = 0
        self.last_ok_t = None
        self.decayed = False

        self._stop_flag = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="commander", daemon=True)
        self._thread.start()

    # ---------------- caller side ----------------

    def set(self, linear, angular):
        """Publish a new setpoint. Never blocks on the network."""
        with self._lock:
            self._setpoint = (linear, angular)
            self._setpoint_t = self.clock()

    def stop(self):
        self.set(0.0, 0.0)

    @property
    def healthy(self):
        """Has a command actually gone out recently?"""
        return self.last_ok_t is not None and self.clock() - self.last_ok_t < 2.0

    def close(self):
        """Stop streaming and make a real effort to leave the rover stationary.

        Called from a `finally`, so it must never raise — an exception here would
        mask whatever went wrong in the control loop.
        """
        if self._closed:
            return
        self._closed = True
        self._stop_flag.set()
        self._thread.join(timeout=2.0)
        for _ in range(self.stop_attempts):
            try:
                self.send(0.0, 0.0)
                break
            except Exception:
                time.sleep(self.stop_gap_s)
        if self.heartbeat_path:
            # Removing it says "clean exit" to the watchdog. A stale file would
            # read as a crash and trigger a stop the rover does not need.
            try:
                os.unlink(self.heartbeat_path)
            except OSError:
                pass

    # ---------------- thread side ----------------

    def _current(self):
        with self._lock:
            (linear, angular), t = self._setpoint, self._setpoint_t
        if self.clock() - t > self.stale_s:
            self.decayed = True
            return 0.0, 0.0
        self.decayed = False
        return linear, angular

    def _touch_heartbeat(self):
        if not self.heartbeat_path:
            return
        try:
            with open(self.heartbeat_path, "w") as f:
                f.write(str(self.clock()))
        except OSError:
            pass

    def _loop(self):
        while not self._stop_flag.is_set():
            started = time.time()
            linear, angular = self._current()
            try:
                self.send(linear, angular)
                self.sent += 1
                self.last_ok_t = self.clock()
                self._touch_heartbeat()
            except Exception:
                self.failures += 1
            self._stop_flag.wait(max(0.0, self.period - (time.time() - started)))
