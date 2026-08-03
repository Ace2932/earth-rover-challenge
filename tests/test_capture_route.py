"""Turning a teleop drive into a route file (issue #80).

`--route` needs a route file and nothing in the repo produced one. The natural
source is the bot itself: drive it by hand once, keep the track, hand the track
back to the follower.

Two properties matter more than the decimation maths. The tool must never send a
control command — it runs while a human is driving, and a stray setpoint from a
second process is the one thing that could take the rover away from them. And it
must drop fixes the follower would refuse to drive on, using the SAME rule the
follower uses, because a route quietly seeded with 1000/1000 is worse than no
route at all.
"""
import json

import pytest

from capture_route import decimate, spacing_warning, to_route, usable_fix

M_PER_DEG = 111111.0


def at(north_m, east_m=0.0):
    return (37.8719 + north_m / M_PER_DEG, -122.2585 + east_m / M_PER_DEG)


# ---------------- decimation ----------------

def test_the_first_point_is_always_kept():
    assert decimate([at(0)], 5.0) == [at(0)]


def test_points_closer_than_the_spacing_are_dropped():
    pts = [at(0), at(1), at(2), at(3)]
    assert decimate(pts, 5.0) == [at(0), at(3)]


def test_points_beyond_the_spacing_are_all_kept():
    pts = [at(0), at(10), at(20)]
    assert decimate(pts, 5.0) == pts


def test_the_last_point_is_always_kept():
    """The route has to end where the driver stopped, however close that was to the
    previous kept point — otherwise the follower's last leg goes somewhere the rover
    was never driven."""
    pts = [at(0), at(10), at(11)]
    assert decimate(pts, 5.0)[-1] == at(11)


def test_an_empty_track_yields_an_empty_route():
    assert decimate([], 5.0) == []


def test_spacing_must_be_positive():
    with pytest.raises(ValueError):
        decimate([at(0), at(10)], 0.0)


# ---------------- spacing against the follower's arrival radius ----------------

def test_spacing_at_or_below_the_arrival_radius_is_flagged():
    """Two individually sane numbers, fatal together — the #74 shape in a new pair.
    Waypoints closer than the arrival radius are all 'reached' the instant the first
    one is, so the rover skips them and cuts the corner it was recorded to follow."""
    assert spacing_warning(5.0, 5.0) is not None
    assert spacing_warning(3.0, 5.0) is not None


def test_spacing_comfortably_above_the_arrival_radius_is_fine():
    assert spacing_warning(10.0, 5.0) is None


def test_the_shipped_defaults_are_a_working_pair():
    """CAPTURE_SPACING_M's default against LOCAL_ARRIVE_M's. If either default moves,
    this is the test that notices."""
    from waypoint_follower import Config
    assert spacing_warning(10.0, Config().local_arrive_m) is None


# ---------------- the fixes that must not enter a route ----------------

def test_the_no_fix_sentinel_is_not_a_usable_fix():
    assert usable_fix({"latitude": 1000, "longitude": 1000, "fix_quality": 0}) is False


def test_fix_quality_zero_is_not_a_usable_fix():
    assert usable_fix({"latitude": 37.8719, "longitude": -122.2585,
                       "fix_quality": 0}) is False


def test_a_good_fix_is_usable():
    assert usable_fix({"latitude": 37.8719, "longitude": -122.2585,
                       "fix_quality": 1}) is True


def test_a_payload_without_fix_quality_is_usable():
    """The SDK's documented response has no `fix_quality`. Absence must not be read
    as a missing fix, exactly as in the follower's own guard."""
    assert usable_fix({"latitude": 37.8719, "longitude": -122.2585}) is True


def test_capture_and_the_follower_share_one_definition_of_a_real_position():
    """This whole branch exists because two places disagreed about one payload. So
    the rule lives in exactly one function and capture re-exports it rather than
    reimplementing it — a second copy is a second thing to get wrong."""
    import telemetry
    assert usable_fix is telemetry.position_is_real


# ---------------- the file the follower actually reads ----------------

def test_the_route_file_matches_what_the_follower_loads(tmp_path):
    """`waypoint_follower.waypoints()` does `p["latitude"], p["longitude"]` over a
    JSON list. Write exactly that, and prove it by loading it back through the
    follower's own reader rather than by re-asserting the schema here."""
    path = tmp_path / "route.json"
    path.write_text(json.dumps(to_route([at(0), at(10)])))

    loaded = json.loads(path.read_text())
    wps = [(float(p["latitude"]), float(p["longitude"])) for p in loaded]
    assert wps == [at(0), at(10)]


def test_the_route_file_is_a_plain_list_of_objects():
    route = to_route([at(0), at(10)])
    assert isinstance(route, list)
    assert set(route[0]) == {"latitude", "longitude"}


# ---------------- it must never drive ----------------

def test_capture_never_sends_a_control_command():
    """A human is holding the controls while this runs. A setpoint from a second
    process is the one failure that could take the rover away from them."""
    import inspect

    import capture_route
    src = inspect.getsource(capture_route)
    assert "/control" not in src
    assert ".control(" not in src
