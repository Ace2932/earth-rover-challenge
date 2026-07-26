"""Telemetry guards (issue #9).

`/data` returns battery, signal_level, gps_signal, speed, rpms, accels, gyros and
mags. The follower read three fields out of it — latitude, longitude, orientation
— and dropped the rest. So it would drive at full cruise on a dying battery, keep
the same confidence in an urban canyon as in open sky, and never notice that a
command produced no wheel motion at all.
"""
from telemetry import Guard
from waypoint_follower import Config, run

M_PER_DEG = 111111.0


def cfg(**kw):
    c = Config()
    c.stuck_s = 30.0
    c.max_runtime_s = 2.0
    c.loop_hz = 20.0
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def data(**kw):
    d = {"battery": 88.0, "signal_level": 5, "gps_signal": 31.0, "speed": 0.9,
         "rpms": [[100, 100, 100, 100]], "latitude": 37.8719, "longitude": -122.2585,
         "orientation": 0}
    d.update(kw)
    return d


# ---------------- battery ----------------

def test_a_healthy_battery_is_not_an_abort():
    g = Guard(cfg())
    v = g.check(data(battery=80.0), cmd_linear=0.6, now=1000.0)
    assert v.abort is None


def test_a_flat_battery_aborts_the_run():
    """Dying mid-mission is a lost mission AND a rover to go and physically fetch."""
    g = Guard(cfg(battery_abort_pct=15.0))
    v = g.check(data(battery=12.0), cmd_linear=0.6, now=1000.0)
    assert v.abort is not None
    assert "battery" in v.abort.lower()


def test_a_low_battery_warns_before_it_aborts():
    g = Guard(cfg(battery_abort_pct=15.0, battery_warn_pct=30.0))
    v = g.check(data(battery=25.0), cmd_linear=0.6, now=1000.0)
    assert v.abort is None
    assert any("battery" in w.lower() for w in v.warnings)


def test_a_missing_battery_field_is_not_an_abort():
    """Never brick a run over a field the SDK did not send."""
    d = data()
    del d["battery"]
    v = Guard(cfg()).check(d, cmd_linear=0.6, now=1000.0)
    assert v.abort is None


# ---------------- gps quality ----------------

def test_good_gps_runs_at_full_speed():
    g = Guard(cfg(gps_signal_good=20.0, gps_signal_poor=5.0))
    assert g.check(data(gps_signal=31.0), cmd_linear=0.6, now=1000.0).speed_scale == 1.0


def test_poor_gps_slows_the_rover_down():
    """Steering on a bad fix at full cruise is how you end up in the road."""
    g = Guard(cfg(gps_signal_good=20.0, gps_signal_poor=5.0, min_speed_scale=0.3))
    assert g.check(data(gps_signal=3.0), cmd_linear=0.6, now=1000.0).speed_scale == 0.3


def test_marginal_gps_scales_between_the_two():
    g = Guard(cfg(gps_signal_good=20.0, gps_signal_poor=5.0, min_speed_scale=0.3))
    scale = g.check(data(gps_signal=12.5), cmd_linear=0.6, now=1000.0).speed_scale
    assert 0.3 < scale < 1.0


def test_gps_scaling_is_disabled_when_thresholds_are_equal():
    """Escape hatch: the units of gps_signal are not documented, so it must be
    possible to switch this off without editing code."""
    g = Guard(cfg(gps_signal_good=0.0, gps_signal_poor=0.0))
    assert g.check(data(gps_signal=0.0), cmd_linear=0.6, now=1000.0).speed_scale == 1.0


# ---------------- commanded vs actual motion ----------------

def test_commanded_motion_with_no_wheel_motion_is_reported():
    """An RTM message that never arrived, or a high-centred rover: either way the
    rover is not doing what it was told, and that is worth knowing before STUCK_S."""
    g = Guard(cfg(no_motion_s=1.0))
    for t in range(5):
        v = g.check(data(speed=0.0, rpms=[[0, 0, 0, 0]]), cmd_linear=0.6, now=1000.0 + t)
    assert v.no_motion is True


def test_no_motion_is_not_reported_while_the_rover_is_actually_moving():
    g = Guard(cfg(no_motion_s=1.0))
    for t in range(5):
        v = g.check(data(speed=0.9), cmd_linear=0.6, now=1000.0 + t)
    assert v.no_motion is False


def test_no_motion_is_not_reported_when_nothing_was_commanded():
    g = Guard(cfg(no_motion_s=1.0))
    for t in range(5):
        v = g.check(data(speed=0.0, rpms=[[0, 0, 0, 0]]), cmd_linear=0.0, now=1000.0 + t)
    assert v.no_motion is False


def test_motion_resets_the_no_motion_timer():
    g = Guard(cfg(no_motion_s=1.0))
    g.check(data(speed=0.0, rpms=[[0, 0, 0, 0]]), cmd_linear=0.6, now=1000.0)
    g.check(data(speed=0.9), cmd_linear=0.6, now=1001.5)
    v = g.check(data(speed=0.0, rpms=[[0, 0, 0, 0]]), cmd_linear=0.6, now=1002.0)
    assert v.no_motion is False


# ---------------- in the control loop ----------------

class FlatBatteryIO:
    hsrc = "stub"

    def __init__(self):
        self.commands = []
        self.last_data = data(battery=5.0)

    def waypoints(self, route_file):
        return [(37.8719 + 500 / M_PER_DEG, -122.2585)], 0

    def get_pose(self):
        return 37.8719, -122.2585, 0.0

    def control(self, linear, angular):
        self.commands.append((linear, angular))

    def front_frame(self):
        return None

    def reached(self):
        return False, {}


def test_the_loop_aborts_and_stops_on_a_flat_battery():
    io = FlatBatteryIO()
    assert run(io, cfg(battery_abort_pct=15.0)) is False
    assert io.commands[-1] == (0, 0)
    assert len(io.commands) < 10           # aborted immediately, did not drive on


def test_the_loop_scales_speed_down_on_poor_gps():
    class PoorGpsIO(FlatBatteryIO):
        def __init__(self):
            super().__init__()
            self.last_data = data(gps_signal=1.0)

    good = FlatBatteryIO()
    good.last_data = data(gps_signal=31.0)
    poor = PoorGpsIO()
    c = cfg(gps_signal_good=20.0, gps_signal_poor=5.0, min_speed_scale=0.3)
    run(good, c)
    run(poor, c)
    assert max(l for l, _ in poor.commands) < max(l for l, _ in good.commands)

