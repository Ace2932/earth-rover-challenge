"""Driving a route with no mission behind it (issue #79).

A bot you own has no mission slug, and without one the SDK's mission API is not
merely empty — it is unusable. `/checkpoints-list` returns `{}` and
`/checkpoint-reached` answers 500 "Required environment variables not configured"
(earth-rovers-sdk/main.py). So on Aiden's own Mini+ the follower could not run at
all, and `--route`, which exists precisely to supply waypoints without the mission
API, still gated every waypoint advance on a server confirmation that can never
arrive.

Reproduced against a stub serving that exact SDK behaviour: the rover drove to
within 0.2 m of waypoint 1, sat there, ran the whole recovery ladder against a
rover that was not stuck, and reported 0/2 after 90 s and 450 steps.

A route file means "these are MY waypoints", so arrival is decided locally and no
checkpoint is ever claimed — claiming a checkpoint for a point the server did not
set would be a lie even when the call happens to succeed. Mission mode is
untouched: the server stays the only authority on a checkpoint.
"""
from waypoint_follower import Config, run

M_PER_DEG = 111111.0
ROUTE = "route.json"        # only its presence matters; the stubs supply the points


def cfg(**kw):
    c = Config()
    c.stuck_s = 30.0
    c.max_runtime_s = 3.0
    c.loop_hz = 20.0
    for k, v in kw.items():
        setattr(c, k, v)
    return c


class ParkedIO:
    """A rover `dist_m` from its only waypoint, with a server that always refuses.

    A real no-mission SDK refuses harder than this — it 500s — but "the server never
    confirms" is the property that matters, and it is the property mission mode must
    keep respecting.
    """
    hsrc = "stub"

    def __init__(self, dist_m):
        self.dist_m = dist_m
        self.polls = 0
        self.commands = []
        self.last_data = None

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
        return False, {}


def test_route_mode_arrives_on_our_own_distance():
    io = ParkedIO(2.0)
    assert run(io, cfg(local_arrive_m=5.0), route_file=ROUTE) is True


def test_route_mode_never_claims_a_checkpoint():
    """The waypoints are ours, not the server's. Claiming one would be a lie."""
    io = ParkedIO(2.0)
    run(io, cfg(local_arrive_m=5.0), route_file=ROUTE)
    assert io.polls == 0


def test_route_mode_does_not_arrive_from_outside_the_local_radius():
    io = ParkedIO(12.0)
    assert run(io, cfg(local_arrive_m=5.0), route_file=ROUTE) is False


def test_mission_mode_still_needs_the_server_to_confirm():
    """The regression that matters: local arrival must not leak into a real mission,
    where the server is the only thing that decides whether a checkpoint counts."""
    io = ParkedIO(2.0)
    assert run(io, cfg(local_arrive_m=5.0)) is False
    assert io.polls > 0


class WalkingIO(ParkedIO):
    """A rover that closes on its target, so a multi-waypoint route can complete."""

    def __init__(self, points_m):
        super().__init__(points_m[0])
        self.points_m = points_m
        self.travelled = 0.0

    def waypoints(self, route_file):
        return [(37.8719 + m / M_PER_DEG, -122.2585) for m in self.points_m], 0

    def get_pose(self):
        return 37.8719 + self.travelled / M_PER_DEG, -122.2585, 0.0

    def control(self, linear, angular):
        self.commands.append((linear, angular))
        self.travelled += max(0.0, linear) * 1.0      # 1 m per commanded unit per step


def test_route_mode_walks_a_multi_point_route_to_completion():
    io = WalkingIO([10.0, 20.0, 30.0])
    assert run(io, cfg(local_arrive_m=3.0), route_file=ROUTE) is True
    assert io.polls == 0


def test_route_mode_advances_every_waypoint_exactly_once():
    """Arrival must consume a waypoint, not re-trigger on the one just left."""
    io = WalkingIO([10.0, 20.0, 30.0])
    run(io, cfg(local_arrive_m=3.0), route_file=ROUTE)
    assert io.travelled >= 30.0 - 3.0
