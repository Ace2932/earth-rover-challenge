"""Small operational gaps (#63 run log durability, #64 config validation, #65 --mock route).

None of these are subtle. All three cost you on the day: a log you cannot read after
a crash, a config typo that drives the rover backwards, and a route check that
silently checks a different route.
"""
import json

import pytest

from waypoint_follower import Config, MockIO, RunLogger


# ---------------- #63: the log has to survive the crash it documents ----------------

def test_rows_reach_disk_before_close(tmp_path):
    """The failures worth investigating are the ones that skip `finally` — kill -9,
    OOM, a suspended laptop. An unflushed log is empty in exactly those cases."""
    p = tmp_path / "run.csv"
    log = RunLogger(str(p))
    for i in range(50):
        log.row(t=float(i), wp=1, lat=37.0, lon=-122.0, heading=0.0, hsrc="dr",
                dist=10.0, sdist="", bearing=0.0, err=0.0, linear=0.5, angular=0.0)
    assert len(p.read_text().strip().split("\n")) == 51      # header + 50, before close()


# ---------------- #64: refuse a config that cannot work ----------------

@pytest.mark.parametrize("key,value,why", [
    ("CRUISE", "-1", "drives backwards at full throttle"),
    ("LOOP_HZ", "0", "ZeroDivisionError in run()"),
    ("STUCK_S", "-5", "instantly stuck, recovery from step one"),
    ("CHECKPOINT_RADIUS_M", "0", "never polls, never claims a checkpoint"),
    ("COMMAND_HZ", "0", "the command streamer never sends"),
    ("MIN_SPEED_SCALE", "5", "a speed cap that multiplies speed"),
    ("BLOCKED_P", "-0.5", "brakes on every frame"),
])
def test_a_mission_killing_value_is_refused(monkeypatch, key, value, why):
    monkeypatch.setenv(key, value)
    with pytest.raises(ValueError) as e:
        Config.from_env()
    assert key in str(e.value), f"error should name {key} ({why})"


@pytest.mark.parametrize("key,value", [
    ("CRUISE", "0.6"), ("LOOP_HZ", "5"), ("STUCK_S", "20"),
    ("CHECKPOINT_RADIUS_M", "20"), ("MIN_SPEED_SCALE", "0.3"),
    ("HEADING_OFFSET", "-45"),          # signed by design
    ("HEADING_SIGN", "-1"),             # signed by design
])
def test_legitimate_values_still_pass(monkeypatch, key, value):
    monkeypatch.setenv(key, value)
    Config.from_env()


def test_the_defaults_validate():
    Config.from_env()


# ---------------- #65: --mock must honour --route ----------------

def test_mock_honours_a_route_file(tmp_path):
    route = tmp_path / "route.json"
    route.write_text(json.dumps([{"latitude": 37.5, "longitude": -122.5},
                                 {"latitude": 37.6, "longitude": -122.6}]))
    io = MockIO((37.8719, -122.2585, 0.0), Config())
    wps, start = io.waypoints(str(route))
    assert wps == [(37.5, -122.5), (37.6, -122.6)]
    assert start == 0


def test_mock_without_a_route_still_uses_its_canned_square():
    io = MockIO((37.8719, -122.2585, 0.0), Config())
    wps, _ = io.waypoints(None)
    assert len(wps) == 3
