"""Command streaming and the safety properties that depend on it (#8, #2).

The control loop used to send one command per iteration, inline, with a 5 s
timeout and 3 retries — so a bad 4G moment blocked a single step for >15 s while
the rover held its last command. And because commands go out over Agora RTM
unacked, an HTTP 200 never meant the bot heard anything.

The Commander streams the latest setpoint on its own thread at 20 Hz, decays a
stale setpoint to zero, and guarantees a stop on close.
"""
import threading
import time

from commander import Commander


class Recorder:
    """Stands in for RoverClient.control, with optional injected failures."""

    def __init__(self, fail_times=0, fail_forever=False):
        self.calls = []
        self.fail_times = fail_times
        self.fail_forever = fail_forever
        self.lock = threading.Lock()

    def __call__(self, linear, angular):
        with self.lock:
            if self.fail_forever or self.fail_times > 0:
                self.fail_times -= 1
                raise RuntimeError("connection reset")
            self.calls.append((linear, angular))

    def snapshot(self):
        with self.lock:
            return list(self.calls)


def wait_for(predicate, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_streams_the_setpoint_repeatedly_without_the_caller_doing_anything():
    rec = Recorder()
    c = Commander(rec, hz=50.0)
    try:
        c.set(0.6, 0.1)
        assert wait_for(lambda: len(rec.snapshot()) >= 10)
        assert all(cmd == (0.6, 0.1) for cmd in rec.snapshot()[1:])
    finally:
        c.close()


def test_a_stale_setpoint_decays_to_zero():
    """The in-process watchdog: if the controller stops updating — blocked on a slow
    request, wedged, whatever — the rover coasts to a stop instead of holding throttle."""
    rec = Recorder()
    c = Commander(rec, hz=50.0, stale_s=0.15)
    try:
        c.set(0.6, 0.0)
        assert wait_for(lambda: (0.6, 0.0) in rec.snapshot())
        assert wait_for(lambda: rec.snapshot()[-1] == (0.0, 0.0))
    finally:
        c.close()


def test_a_fresh_setpoint_revives_a_decayed_stream():
    rec = Recorder()
    c = Commander(rec, hz=50.0, stale_s=0.15)
    try:
        c.set(0.6, 0.0)
        assert wait_for(lambda: rec.snapshot()[-1] == (0.0, 0.0))
        c.set(0.4, -0.2)
        assert wait_for(lambda: rec.snapshot()[-1] == (0.4, -0.2))
    finally:
        c.close()


def test_set_does_not_block_on_a_slow_link():
    slow = lambda linear, angular: time.sleep(0.2)
    c = Commander(slow, hz=20.0)
    try:
        t0 = time.time()
        for _ in range(20):
            c.set(0.5, 0.0)
        assert time.time() - t0 < 0.05
    finally:
        c.close()


def test_send_failures_do_not_kill_the_stream():
    rec = Recorder(fail_times=5)
    c = Commander(rec, hz=50.0)
    try:
        c.set(0.5, 0.0)
        assert wait_for(lambda: len(rec.snapshot()) >= 3)
        assert c.failures >= 5
    finally:
        c.close()


def test_close_stops_the_rover():
    rec = Recorder()
    c = Commander(rec, hz=50.0)
    c.set(0.8, 0.3)
    assert wait_for(lambda: (0.8, 0.3) in rec.snapshot())
    c.close()
    assert rec.snapshot()[-1] == (0.0, 0.0)


def test_close_retries_the_stop_when_the_first_attempts_fail():
    """A stop that fails silently is worse than no stop at all."""
    rec = Recorder(fail_times=4)
    c = Commander(rec, hz=50.0, stop_attempts=10, stop_gap_s=0.01)
    c.set(0.8, 0.0)
    time.sleep(0.05)
    c.close()
    assert (0.0, 0.0) in rec.snapshot()


def test_close_is_idempotent():
    rec = Recorder()
    c = Commander(rec, hz=50.0)
    c.close()
    c.close()
    assert rec.snapshot()[-1] == (0.0, 0.0)


def test_close_never_raises_even_when_every_send_fails():
    """Called from a `finally`, so raising here would mask the original exception."""
    rec = Recorder(fail_forever=True)
    c = Commander(rec, hz=50.0, stop_attempts=3, stop_gap_s=0.01)
    c.set(0.5, 0.0)
    c.close()                                  # must not raise


def test_run_closes_the_io_so_the_hardened_stop_runs():
    """The control loop's `finally` must reach Commander.close(), not just fire one
    best-effort control(0,0) that can itself fail."""
    from waypoint_follower import Config, run

    class IO:
        hsrc = "stub"
        closed = False

        def waypoints(self, route_file):
            return [(37.8719, -122.2585)]

        def get_pose(self):
            return 37.8719, -122.2585, 0.0

        def control(self, linear, angular):
            pass

        def front_frame(self):
            return None

        def reached(self):
            return True, {}

        def close(self):
            IO.closed = True

    cfg = Config()
    cfg.max_runtime_s = 1.0
    run(IO(), cfg)
    assert IO.closed is True


def test_the_control_loop_holds_its_rate_rather_than_sleeping_a_fixed_period():
    """`sleep(period)` after the work makes the real rate period + work. With the
    loop deliberately slowed, a deadline-based sleep still lands close to target."""
    import time as _time
    from waypoint_follower import Config, run

    class SlowIO:
        hsrc = "stub"

        def __init__(self):
            self.steps = 0

        def waypoints(self, route_file):
            return [(37.9, -122.2)]                 # far away, never reached

        def get_pose(self):
            self.steps += 1
            _time.sleep(0.02)                       # 20 ms of "work" per step
            return 37.8719, -122.2585, 0.0

        def control(self, linear, angular):
            pass

        def front_frame(self):
            return None

        def reached(self):
            return False, {}

    cfg = Config()
    cfg.loop_hz = 20.0                              # 50 ms period, 20 ms of work
    cfg.max_runtime_s = 1.0
    cfg.stuck_s = 99.0
    io = SlowIO()
    run(io, cfg)
    assert io.steps >= 16                           # ~20 with deadlines, ~14 without


def test_heartbeat_is_refreshed_while_streaming_and_removed_on_close(tmp_path):
    hb = tmp_path / "follower.hb"
    rec = Recorder()
    c = Commander(rec, hz=50.0, heartbeat_path=str(hb))
    try:
        c.set(0.3, 0.0)
        assert wait_for(hb.exists)
        first = hb.stat().st_mtime_ns
        assert wait_for(lambda: hb.stat().st_mtime_ns > first)
    finally:
        c.close()
    assert not hb.exists()                     # clean shutdown, not a crash


def test_live_io_records_the_command_for_the_heading_estimator():
    """Regression for #29. tests/test_heading.py drives HeadingEstimator directly, so
    it cannot see LiveIO forgetting to record the command — and without that record
    the estimator's motion gate never opens and no GPS course is ever accepted."""
    from waypoint_follower import Config, LiveIO

    io = LiveIO.__new__(LiveIO)          # no network, no Commander thread
    io.last_cmd = (0.0, 0.0)
    io.cmd = type("Stub", (), {"set": lambda self, lin, ang: None})()
    LiveIO.control(io, 0.5, -0.2)
    assert io.last_cmd == (0.5, -0.2)
