"""Cross-feature failures: each feature correct alone, wrong together.

This is the defect shape this codebase keeps producing, and the one the per-feature
suites are structurally blind to — every test in them exercises one subsystem with
the others held still.

Covers #71 (recovery reversing vs the heading filter) and #72 (recovery bypassing
the telemetry guards), plus combinations that work today and should stay working.
"""
import math

import pytest

from geo import wrap180
from heading import HeadingEstimator
from telemetry import Guard
from waypoint_follower import Config, run

M_PER_DEG = 111111.0


def cfg(**kw):
    c = Config()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


# ---------------- #71: recovery reverses; the heading filter must not believe it ----------------

def test_reversing_never_produces_a_heading_fix():
    """The chord points backwards along the heading while reversing, so a course
    taken then reports the rover facing 180 deg from reality — the worst error
    available. Safe at default RECOVERY_REVERSE_S only by a factor of ~10, and that
    parameter is exposed in .env.example."""
    est = HeadingEstimator(cfg())
    lat, lon, t, truth = 37.8719, -122.2585, 1000.0, 0.0
    est.update(lat, lon, orientation=0, now=t, cmd_linear=0.0, speed=0.0)
    for _ in range(300):                       # 60 s of reversing: far past the baseline
        lat += (-0.35 * 1.5 / 5) * math.cos(math.radians(truth)) / M_PER_DEG
        t += 0.2
        h, src = est.update(lat, lon, orientation=0, now=t,
                            cmd_linear=-0.35, speed=-0.525)
        assert src != "gps", "took a heading fix from motion it knew was backwards"
    assert abs(wrap180(h - truth)) < 5.0


def test_forward_motion_after_a_reverse_still_gets_fixes():
    """Rejecting reverse must not poison the filter for the rest of the run."""
    est = HeadingEstimator(cfg())
    lat, lon, t = 37.8719, -122.2585, 1000.0
    est.update(lat, lon, orientation=0, now=t, cmd_linear=0.0, speed=0.0)
    for _ in range(10):                        # a short reverse, as recovery does
        lat -= (0.35 * 1.5 / 5) / M_PER_DEG
        t += 0.2
        est.update(lat, lon, orientation=0, now=t, cmd_linear=-0.35, speed=-0.525)
    fixes = 0
    for _ in range(400):                       # then drive forward normally
        lat += (0.9 / 5) / M_PER_DEG
        t += 0.2
        _, src = est.update(lat, lon, orientation=0, now=t, cmd_linear=0.6, speed=0.9)
        fixes += src == "gps"
    assert fixes > 3


# ---------------- #72: the guards must not switch off during a manoeuvre ----------------

class RecoveringIO:
    """A wedged rover, so the recovery ladder always runs. `payload` is whatever
    /data reports — the test controls battery and timestamp through it."""

    hsrc = "stub"

    def __init__(self, payload):
        self.last_data = payload
        self.commands = []
        self.interventions = []

    def waypoints(self, route_file):
        return [(37.8719 + 300 / M_PER_DEG, -122.2585)], 0

    def get_pose(self):
        return 37.8719, -122.2585, 0.0

    def control(self, linear, angular):
        self.commands.append((linear, angular))

    def front_frame(self):
        return None, None

    def reached(self):
        return False, {}

    def intervention(self, action):
        self.interventions.append(action)


def payload(**kw):
    d = {"battery": 88.0, "signal_level": 5, "gps_signal": 31.0, "speed": 0.0,
         "rpms": [[0, 0, 0, 0]], "timestamp": 1000.0}
    d.update(kw)
    return d


def test_a_flat_battery_aborts_even_mid_manoeuvre():
    """The battery floor exists to leave the rover somewhere retrievable. Waiting for
    three back-up-and-turn attempts to finish defeats the point."""
    io = RecoveringIO(payload(battery=5.0))
    assert run(io, cfg(stuck_s=0.3, max_runtime_s=4.0, battery_abort_pct=15.0,
                       loop_hz=20.0)) is False
    assert len(io.commands) < 40, "kept manoeuvring on a flat battery"


def test_a_frozen_fix_stops_the_recovery_ladder():
    """#59 exists to stop the rover driving on a fix that is no longer real, and its
    own issue text named 'a recovery ladder planned against fiction' as the danger.
    Recovery was exactly where that check was skipped."""
    io = RecoveringIO(payload())                       # timestamp never advances
    run(io, cfg(stuck_s=0.3, max_runtime_s=4.0, fix_max_age_s=0.5,
                max_consecutive_errors=6, loop_hz=20.0))
    reversing = [c for c in io.commands if c[0] < 0]
    assert not reversing, "ran the back-up manoeuvre on a frozen position fix"
    assert io.commands[-1] == (0, 0)


def test_recovery_still_runs_when_everything_is_healthy():
    """The guards must not have disabled the ladder outright."""
    io = RecoveringIO(payload())
    ticks = {"n": 0}
    real = io.get_pose

    def advancing_pose():
        ticks["n"] += 1
        io.last_data = payload(timestamp=1000.0 + ticks["n"])   # a live fix
        return real()

    io.get_pose = advancing_pose
    run(io, cfg(stuck_s=0.3, max_runtime_s=4.0, recovery_tries=2, loop_hz=20.0))
    assert any(c[0] < 0 for c in io.commands), "the ladder never backed up"


# ---------------- config values that are fatal only in combination (#74) ----------------

def test_a_loop_slower_than_the_setpoint_staleness_is_refused(monkeypatch):
    """The commander decays a setpoint nobody refreshed. If the loop period exceeds
    that window the setpoint expires between every iteration: measured at LOOP_HZ=1
    with the default 0.5 s staleness, 52% of streamed commands were zero throttle."""
    monkeypatch.setenv("LOOP_HZ", "1")
    with pytest.raises(ValueError) as e:
        Config.from_env()
    msg = str(e.value)
    assert "LOOP_HZ" in msg and "SETPOINT_STALE_S" in msg, (
        f"the message must name both, since neither is wrong alone: {msg}")


def test_raising_the_staleness_makes_a_slow_loop_legal(monkeypatch):
    """The constraint is a relationship, not a ban on slow loops."""
    monkeypatch.setenv("LOOP_HZ", "1")
    monkeypatch.setenv("SETPOINT_STALE_S", "3")
    Config.from_env()


def test_the_defaults_satisfy_the_relationship():
    Config.from_env()


def test_a_fast_loop_is_unaffected(monkeypatch):
    monkeypatch.setenv("LOOP_HZ", "20")
    Config.from_env()


def test_the_watchdog_timeout_follows_the_command_rate():
    """The heartbeat interval is 1/COMMAND_HZ. A timeout shorter than that stops a
    perfectly healthy rover — measured at COMMAND_HZ=0.5, 14 stops in 6 s."""
    from waypoint_follower import watchdog_timeout_s

    assert watchdog_timeout_s(cfg(command_hz=20.0)) == pytest.approx(1.0)
    slow = watchdog_timeout_s(cfg(command_hz=0.5))
    assert slow > 2.0, "must exceed the 2 s heartbeat interval"
    assert slow <= 12.0, "but must still detect a dead follower in reasonable time"


def test_the_watchdog_timeout_never_drops_below_the_default():
    """A very fast streamer must not shrink the window so far that ordinary jitter
    reads as death."""
    from waypoint_follower import watchdog_timeout_s

    assert watchdog_timeout_s(cfg(command_hz=200.0)) >= 1.0
