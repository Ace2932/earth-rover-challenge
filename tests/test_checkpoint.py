"""Server-authoritative checkpoint distance (issue #3).

The challenge accepts a checkpoint within 15 m, and the SDK's 400 response carries
`proximate_distance_to_checkpoint` — its own answer to "how far am I". The follower
asked only when its locally computed distance dropped under 5 m, and then discarded
the payload. With a GPS bias, the local distance never gets there, so the server is
never asked about a checkpoint it would have accepted, and the run dies of stuck
detection at a checkpoint it had already reached.
"""
import math

from rover_client import server_distance
from waypoint_follower import Config, run

M_PER_DEG = 111111.0


def cfg(**kw):
    c = Config()
    c.stuck_s = 1.5
    c.max_runtime_s = 3.0
    c.loop_hz = 20.0
    for k, v in kw.items():
        setattr(c, k, v)
    return c


class ParkedIO:
    """A rover sitting `dist_m` from its only checkpoint, ignoring commands.

    `accept` decides whether the server confirms the checkpoint; when it refuses it
    answers with the SDK's real 400 body, reporting `server_dist_m`.
    """

    hsrc = "stub"

    def __init__(self, dist_m, accept=True, server_dist_m=None):
        self.dist_m = dist_m
        self.accept = accept
        self.server_dist_m = dist_m if server_dist_m is None else server_dist_m
        self.polls = 0
        self.steps = 0
        self.commands = []

    def waypoints(self, route_file):
        # NOTE: PR #19 changes this contract to (waypoints, start_index); after both
        # land this returns ([...], 0).
        return [(37.8719 + self.dist_m / M_PER_DEG, -122.2585)]

    def get_pose(self):
        self.steps += 1
        return 37.8719, -122.2585, 0.0

    def control(self, linear, angular):
        self.commands.append((linear, angular))

    def front_frame(self):
        return None

    def reached(self):
        self.polls += 1
        if self.accept:
            return True, {"message": "Checkpoint reached successfully"}
        return False, {"detail": {
            "error": "Bot is not within 15 meters from the checkpoint",
            "proximate_distance_to_checkpoint": self.server_dist_m}}


# ---------------- parsing the SDK's 400 ----------------

def test_server_distance_reads_the_sdk_400_body():
    detail = {"detail": {"error": "Bot is not within 15 meters from the checkpoint",
                         "proximate_distance_to_checkpoint": 12.5}}
    assert server_distance(detail) == 12.5


def test_server_distance_is_none_on_success():
    assert server_distance({"message": "Checkpoint reached successfully"}) is None


def test_server_distance_survives_a_plain_string_detail():
    assert server_distance({"detail": "Bot unavailable for SDK"}) is None


def test_server_distance_survives_a_transport_error_payload():
    assert server_distance({"error": "connection refused"}) is None


# ---------------- polling gate ----------------

def test_asks_the_server_inside_the_poll_radius_even_when_the_local_gate_is_unmet():
    """12 m out: the old 5 m gate never asked, so a checkpoint the server accepts
    at 15 m was never claimed."""
    io = ParkedIO(dist_m=12.0, accept=True)
    assert run(io, cfg()) is True
    assert io.polls >= 1


def test_does_not_ask_the_server_from_far_away():
    io = ParkedIO(dist_m=120.0, accept=True)
    run(io, cfg())
    assert io.polls == 0


def test_polling_is_rate_limited():
    """One request per second, not one per control step — the loop runs at 20 Hz here."""
    io = ParkedIO(dist_m=12.0, accept=False)
    run(io, cfg(checkpoint_poll_s=1.0))
    assert io.steps > 20
    assert io.polls <= 4


# ---------------- using the answer ----------------

def test_server_distance_drives_the_approach_slowdown():
    """Local geometry says 12 m (full cruise); the server says 1 m. Creep."""
    far = ParkedIO(dist_m=12.0, accept=False, server_dist_m=12.0)
    near = ParkedIO(dist_m=12.0, accept=False, server_dist_m=1.0)
    run(far, cfg())
    run(near, cfg())
    assert max(l for l, _ in near.commands) < max(l for l, _ in far.commands)


def test_stuck_detection_uses_the_server_distance():
    """Closing on the checkpoint by the server's own measure is progress, even if the
    local fix disagrees — so it must not be declared stuck."""
    class ClosingIO(ParkedIO):
        def reached(self):
            self.polls += 1
            self.server_dist_m = max(0.0, self.server_dist_m - 3.0)
            return False, {"detail": {"error": "not within 15 meters",
                                      "proximate_distance_to_checkpoint": self.server_dist_m}}

    io = ClosingIO(dist_m=14.0, accept=False, server_dist_m=14.0)
    run(io, cfg(stuck_s=1.5, max_runtime_s=3.0, checkpoint_poll_s=0.3))
    assert io.polls >= 4                      # kept polling rather than aborting early
