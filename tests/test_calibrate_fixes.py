"""Calibration must not calibrate against a position that does not exist (issue #82).

`calibrate_heading.py` derives HEADING_OFFSET from the bearing between the first and
last GPS sample of a short straight drive. It never checked whether those samples
were real fixes.

The all-sentinel case was already safe by luck: 1000/1000 throughout gives a
haversine of 0 and trips the existing "moved only 0.00 m" guard. The dangerous case
is the MIXED one — a lock that arrives, or drops, partway through the 4-second
drive. Then the chord runs between a real position and 1000/1000, the distance is
enormous so the guard waves it through, and the tool prints a confident
HEADING_OFFSET derived from a bearing to nowhere.

A wrong calibration is worse than no calibration: it is the seed the heading filter
starts from, and it is exported by hand into the environment of every run that day.
"""
import pytest

from calibrate_heading import collect, usable_samples


def sample(lat, lon=-122.2585, orientation=64.0):
    return (lat, lon, orientation)


# ---------------- which samples may be calibrated against ----------------

def test_the_no_fix_sentinel_is_dropped():
    got = usable_samples([sample(1000.0, 1000.0), sample(37.8719), sample(37.8720)])
    assert got == [sample(37.8719), sample(37.8720)]


def test_a_lock_that_drops_midway_is_dropped():
    """The dangerous case: real, real, sentinel. The chord would otherwise run from a
    real position to nowhere, and the distance guard would wave it through."""
    got = usable_samples([sample(37.8719), sample(37.8720), sample(1000.0, 1000.0)])
    assert got == [sample(37.8719), sample(37.8720)]


def test_good_samples_are_all_kept():
    pts = [sample(37.8719), sample(37.8720), sample(37.8721)]
    assert usable_samples(pts) == pts


def test_an_all_sentinel_run_yields_nothing():
    assert usable_samples([sample(1000.0, 1000.0)] * 5) == []


def test_calibration_and_the_follower_share_one_definition_of_a_real_position():
    """Third consumer of the same rule, and still only one copy of it."""
    import inspect

    import calibrate_heading
    assert "position_is_real" in inspect.getsource(calibrate_heading.usable_samples)


# ---------------- the drive itself is unchanged ----------------

class SentinelClient:
    """A bot driving with no GPS lock: it still moves, it just cannot say where."""

    def __init__(self):
        self.commands = []

    def get_data(self):
        return {"latitude": 1000, "longitude": 1000, "orientation": 64,
                "fix_quality": 0}

    def control(self, linear, angular):
        self.commands.append((linear, angular))


def test_collect_still_stops_the_rover_when_every_fix_is_a_sentinel():
    """Whatever we decide about the samples, the rover must be stopped afterwards —
    a human is standing next to it. #32 must not regress."""
    c = SentinelClient()
    collect(c, secs=0.2, hz=20.0, throttle=0.5)
    assert c.commands[-1] == (0, 0)
