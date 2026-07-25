"""Surviving a bad link (issue #40).

The bot is on 4G. Transient request failures are the normal case, not the
exceptional one — but a single failure that outlived RoverClient's retries used to
propagate straight out of run() and kill the mission, and the safety stop in the
`finally` went through the same failing client, so it raised too and left the
rover with its last command.
"""
import pytest

from waypoint_follower import Config, run

M_PER_DEG = 111111.0


def cfg(**kw):
    c = Config()
    c.max_runtime_s = 2.0
    c.loop_hz = 50.0
    c.stuck_s = 99.0
    for k, v in kw.items():
        setattr(c, k, v)
    return c


class FlakyIO:
    """Fails every `fail_every`-th telemetry read, and optionally every command."""

    hsrc = "stub"

    def __init__(self, fail_every=3, fail_control=0, fail_stop_times=0):
        self.fail_every = fail_every
        self.fail_control = fail_control      # fail every Nth command (0 = never)
        self.fail_stop_times = fail_stop_times
        self.polls = 0
        self.commands = []
        self.pose_calls = 0
        self.control_calls = 0

    def waypoints(self, route_file):
        return [(37.8719 + 300 / M_PER_DEG, -122.2585)], 0

    def get_pose(self):
        self.pose_calls += 1
        if self.fail_every and self.pose_calls % self.fail_every == 0:
            raise RuntimeError("503 Server Error: /data")
        return 37.8719, -122.2585, 0.0

    def control(self, linear, angular):
        if linear == 0 and angular == 0 and self.fail_stop_times:
            self.fail_stop_times -= 1
            raise RuntimeError("503 Server Error: /control")
        self.control_calls += 1
        if self.fail_control and self.control_calls % self.fail_control == 0:
            raise RuntimeError("503 Server Error: /control")
        self.commands.append((linear, angular))

    def front_frame(self):
        return None

    def reached(self):
        return False, {}


def test_a_transient_telemetry_failure_does_not_end_the_run():
    io = FlakyIO(fail_every=3)
    run(io, cfg())
    assert io.pose_calls > 20, "gave up after the first failure"


def test_transient_command_failures_do_not_end_the_run():
    """Every third command is lost — annoying, not fatal."""
    io = FlakyIO(fail_every=0, fail_control=3)
    run(io, cfg())
    assert io.pose_calls > 20


def test_a_link_that_never_recovers_gives_up_deliberately():
    """Failing forever is not 'transient'. Stop, do not spin until max runtime."""
    io = FlakyIO(fail_every=1)
    assert run(io, cfg(max_consecutive_errors=5, max_runtime_s=30.0)) is False
    assert io.pose_calls < 40, "kept hammering a dead link"


def test_the_run_still_stops_the_rover_when_the_link_is_bad():
    io = FlakyIO(fail_every=3)
    run(io, cfg())
    assert io.commands[-1] == (0, 0)


def test_a_failing_stop_is_retried_rather_than_raising():
    """The finally must never raise: it would mask the real failure AND leave the
    rover moving. This is the exact traceback from the 30%-fault-rate run."""
    io = FlakyIO(fail_every=0, fail_stop_times=4)
    run(io, cfg(stop_attempts=10))
    assert (0, 0) in io.commands


def test_a_stop_that_can_never_succeed_does_not_propagate():
    class DeadIO(FlakyIO):
        def control(self, linear, angular):
            raise RuntimeError("link is gone")

    run(DeadIO(fail_every=0), cfg(stop_attempts=3))   # must not raise
