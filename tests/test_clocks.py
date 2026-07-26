"""Durations must not be measured with the wall clock (issue #67).

`DEPLOYMENT.md` recommends running on a freshly booted cloud VM, which is exactly
when a large NTP correction is most likely. A forward step instantly trips
`max_runtime_s` and stuck detection; a backward step freezes the commander's
stale-setpoint decay — the one guarantee that machinery exists to provide.
"""
import os
import re
import time

from commander import Commander
from health import StaleDetector
from watchdog import Watchdog

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# fake_sdk_server stands in for the bot and must emit wall-clock timestamps.
CONTROL_MODULES = ["waypoint_follower.py", "commander.py", "watchdog.py", "health.py",
                   "heading.py", "telemetry.py", "recovery.py", "blocked.py",
                   "calibrate_heading.py"]


def test_no_duration_is_measured_with_the_wall_clock():
    """A wall-clock use must be deliberate and annotated, because the default is
    wrong for every duration in a control loop."""
    offenders = []
    for name in CONTROL_MODULES:
        for n, line in enumerate(open(os.path.join(REPO, name)), 1):
            if "time.time()" in line and "wall-clock:" not in line:
                offenders.append(f"{name}:{n}: {line.strip()}")
    assert not offenders, "un-annotated wall-clock use:\n  " + "\n  ".join(offenders)


def test_the_commander_defaults_to_a_monotonic_clock():
    c = Commander(lambda lin, ang: None, hz=50.0)
    try:
        assert c.clock is time.monotonic
    finally:
        c.close()


def test_the_stale_detector_defaults_to_a_monotonic_clock():
    assert StaleDetector().clock is time.monotonic


# ---------------- the watchdog is the most exposed: it compares a file mtime ----------------

class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def test_a_wall_clock_jump_does_not_trigger_a_spurious_stop():
    """The heartbeat's mtime is stamped by the OS with the wall clock. If that clock
    steps forward 60 s, `now - mtime` says the follower has been dead for a minute —
    and the watchdog would brake a perfectly healthy rover."""
    mono, mtime = Clock(1000.0), [500.0]
    stops = []
    w = Watchdog(stop=lambda: stops.append(1), heartbeat_mtime=lambda: mtime[0],
                 clock=mono, timeout_s=1.0)
    w.tick()
    for _ in range(5):
        mono.t += 0.2
        mtime[0] += 60.0            # the wall clock jumped; the heartbeat is alive
        w.tick()
    assert stops == []


def test_a_backwards_wall_clock_does_not_blind_the_watchdog():
    """Stepping the wall clock back makes `now - mtime` negative, so a naive check
    never fires — the safety backstop goes silent exactly when it is needed."""
    mono, mtime = Clock(1000.0), [5000.0]      # mtime far in the "future"
    stops = []
    w = Watchdog(stop=lambda: stops.append(1), heartbeat_mtime=lambda: mtime[0],
                 clock=mono, timeout_s=1.0)
    w.tick()
    mono.t += 5.0                              # heartbeat frozen for 5 s
    w.tick()
    assert stops, "did not stop the rover on a frozen heartbeat"


def test_an_advancing_heartbeat_is_still_healthy():
    mono, mtime = Clock(1000.0), [1000.0]
    stops = []
    w = Watchdog(stop=lambda: stops.append(1), heartbeat_mtime=lambda: mtime[0],
                 clock=mono, timeout_s=1.0)
    for _ in range(10):
        mono.t += 0.2
        mtime[0] += 0.2
        w.tick()
    assert stops == []
