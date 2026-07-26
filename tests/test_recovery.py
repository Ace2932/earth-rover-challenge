"""Stuck recovery (issue #4).

Stuck detection used to `break` the control loop: the first curb, pole, wet leaf
or GPS glitch ended the mission. Across six remote rounds plus a final, that is a
guaranteed loss of points.

The ladder now is: back up and turn (a few times, alternating direction) ->
approach the checkpoint from a different angle -> only then record an
intervention and stop.
"""
from recovery import Recovery
from waypoint_follower import Config, run

M_PER_DEG = 111111.0


def cfg(**kw):
    c = Config()
    c.stuck_s = 0.4
    c.max_runtime_s = 6.0
    c.loop_hz = 20.0
    c.recovery_pause_s = 0.05
    c.recovery_reverse_s = 0.1
    c.recovery_yaw_s = 0.1
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def phases(rec, t0=1000.0, dt=0.02, n=60):
    """Walk the state machine and collect (linear, angular, active) per tick."""
    out = []
    t = t0
    for _ in range(n):
        out.append(rec.step(t))
        t += dt
    return out


# ---------------- the manoeuvre ----------------

def test_recovery_pauses_then_reverses_then_yaws():
    c = cfg()
    rec = Recovery(c)
    rec.begin(1000.0)
    seq = phases(rec, n=int((c.recovery_pause_s + c.recovery_reverse_s
                             + c.recovery_yaw_s) / 0.02) + 4)
    reversing = [s for s in seq if s[0] < 0]
    yawing = [s for s in seq if s[0] == 0 and s[1] != 0]
    assert reversing, "never backed up"
    assert yawing, "never turned"
    assert seq.index(reversing[0]) < seq.index(yawing[0])
    assert seq[-1][2] is False                      # manoeuvre finished


def test_recovery_is_inactive_until_it_begins():
    rec = Recovery(cfg())
    assert rec.step(1000.0) == (0.0, 0.0, False)


def test_successive_attempts_turn_the_other_way():
    """If turning left did not free it, try right."""
    c = cfg()
    rec = Recovery(c)
    yaws = []
    t = 1000.0
    for _ in range(2):
        rec.begin(t)
        while True:
            lin, ang, active = rec.step(t)
            if not active:
                break
            if lin == 0 and ang != 0:
                yaws.append(ang)
            t += 0.02
        t += 1.0
    assert yaws, "no yaw commands"
    assert yaws[0] * yaws[-1] < 0


def test_recovery_reports_exhausted_after_the_configured_number_of_tries():
    c = cfg(recovery_tries=2)
    rec = Recovery(c)
    t = 1000.0
    for _ in range(2):
        assert rec.exhausted is False
        rec.begin(t)
        while rec.step(t)[2]:
            t += 0.02
        t += 1.0
    assert rec.exhausted is True


# ---------------- in the control loop ----------------

class WedgedIO:
    """A rover that never moves, however hard it is commanded."""

    hsrc = "stub"

    def __init__(self, dist_m=30.0):
        self.dist_m = dist_m
        self.commands = []
        self.polls = 0
        self.poll_marks = []
        self.interventions = []
        self.closed = False

    def waypoints(self, route_file):
        return [(37.8719 + self.dist_m / M_PER_DEG, -122.2585)], 0

    def get_pose(self):
        return 37.8719, -122.2585, 0.0

    def control(self, linear, angular):
        self.commands.append((linear, angular))

    def front_frame(self):
        return None

    def reached(self):
        self.polls += 1
        self.poll_marks.append(len(self.commands))   # when, in command-count terms
        return False, {}

    def intervention(self, action):
        self.interventions.append(action)

    def close(self):
        self.closed = True


def test_being_stuck_triggers_a_recovery_instead_of_ending_the_run():
    io = WedgedIO()
    run(io, cfg())
    assert any(lin < 0 for lin, _ in io.commands), "never tried backing up"


def test_recovery_runs_more_than_once_before_giving_up():
    io = WedgedIO()
    run(io, cfg(recovery_tries=3))
    # each attempt ends with a yaw; count the transitions into reverse
    reverses = sum(1 for k in range(1, len(io.commands))
                   if io.commands[k][0] < 0 <= io.commands[k - 1][0])
    assert reverses >= 2


def test_an_unrecoverable_rover_records_an_intervention_and_stops():
    io = WedgedIO()
    assert run(io, cfg(recovery_tries=1, recovery_offset_m=0.0)) is False
    assert "start" in io.interventions
    assert io.commands[-1] == (0, 0)


def test_the_detour_waypoint_is_never_claimed_as_a_checkpoint():
    """Approaching from a different angle means driving to a point that is NOT a
    checkpoint — asking the server about it would be claiming one we never reached.

    The rover sits 30 m out with a 50 m poll radius, so it polls on every step
    while heading for the checkpoint. Once the detour starts, polling must stop
    dead: the remaining commands are issued with no further polls behind them.
    stuck_s is 1.5 s here so the detour phase lasts ~30 steps at 20 Hz — long
    enough that "no polls" cannot be a coincidence.
    """
    io = WedgedIO()
    run(io, cfg(recovery_tries=1, recovery_offset_m=8.0, checkpoint_radius_m=50.0,
                stuck_s=1.5, max_runtime_s=8.0))
    assert io.polls > 0, "never polled for the real checkpoint"
    commands_after_last_poll = len(io.commands) - io.poll_marks[-1]
    assert commands_after_last_poll > 20, (
        f"only {commands_after_last_poll} commands after the last poll — the detour "
        f"phase looks like it kept claiming the checkpoint")
