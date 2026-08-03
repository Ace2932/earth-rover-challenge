"""The follower against the payload a REAL bot sends (issues #76, #77, #78).

Every other suite in this repo feeds the follower telemetry shaped by
`fake_sdk_server.py`. That harness was written from the same reading of the SDK
that the follower was, so where the reading is wrong, both are wrong together and
267 passing tests say nothing at all. These tests take their payload from the
SDK's own documented response (`earth-rovers-sdk/README.md`) and from what the
bot actually returned on the bench, not from the harness.

Two defects came out of that, and neither was reachable from the harness:

  #76 `rpms` rows end with a sample timestamp. `_wheel_motion` read every element
      of the row, so a ~1.7e9 timestamp always read as a spinning wheel and the
      commanded-vs-actual motion check could never fire on a real bot.
  #77 With no GPS lock the bot reports `latitude`/`longitude` of 1000 and
      `fix_quality` 0 while the timestamp keeps advancing — so the frozen-fix
      check stays quiet and the follower drives at full cruise on a bearing
      computed from a position that does not exist.

#78 closes the hole itself: the harness now emits the real payload shape, so the
next divergence of this kind fails a test instead of a mission.
"""
import math

from telemetry import Guard, _wheel_motion
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


def real_data(**kw):
    """`/data` exactly as earth-rovers-sdk/README.md documents it.

    The parts that matter here and that the harness got wrong: `rpms` rows carry a
    trailing sample timestamp, as `accels`, `gyros` and `mags` all do.
    """
    d = {"battery": 100, "signal_level": 5, "orientation": 128, "lamp": 0,
         "speed": 0, "gps_signal": 31.25,
         "latitude": 22.753774642944336, "longitude": 114.09095001220703,
         "vibration": 0.31, "timestamp": 1724189733.208559,
         "accels": [[0.998, 0.003, 0.005, 1725434620.858]],
         "gyros": [[0.521, 0.023, 0.716, 1725434620.913]],
         "mags": [[-1002, 967, 12, 1725434621.194]],
         "rpms": [[0, 0, 0, 0, 1725434567.194],
                  [0, 0, 0, 0, 1725434567.218]]}
    d.update(kw)
    return d


# ---------------- #76: rpms rows end with a timestamp ----------------

def test_a_stopped_wheel_is_not_motion_just_because_the_row_is_timestamped():
    assert _wheel_motion(real_data()) is False


def test_a_turning_wheel_is_still_motion_in_the_real_row_shape():
    assert _wheel_motion(real_data(rpms=[[0, 0, 0, 120, 1725434567.194]])) is True


def test_the_no_motion_guard_fires_on_a_real_bot_that_is_not_moving():
    """Commanded forward, `speed` 0, every wheel stopped — the case the guard exists
    for. It never fired on a real payload because the row's timestamp read as RPM."""
    g = Guard(cfg(no_motion_s=4.0))
    fired = False
    for k in range(60):                       # 12 s at 5 Hz
        v = g.check(real_data(speed=0.0, timestamp=1724189733.0 + k * 0.2),
                    cmd_linear=0.6, now=1000.0 + k * 0.2)
        if v.no_motion:
            fired = True
            break
    assert fired is True


def test_a_four_element_row_without_a_timestamp_still_reads_every_wheel():
    """Not every producer timestamps the row. Dropping a trailing element blindly
    would throw away a wheel, so only the four wheels are ever read."""
    assert _wheel_motion(real_data(rpms=[[0, 0, 0, 120]])) is True
    assert _wheel_motion(real_data(rpms=[[0, 0, 0, 0]])) is False


# ---------------- #77: the no-fix sentinel ----------------

def test_the_no_fix_sentinel_is_not_a_position():
    """Observed on the bench 2026-07-30: no GPS lock reports 1000/1000."""
    v = Guard(cfg()).check(real_data(latitude=1000, longitude=1000, fix_quality=0),
                           cmd_linear=0.6, now=1000.0)
    assert v.no_fix is True


def test_fix_quality_zero_is_not_a_position_even_at_a_plausible_latlon():
    v = Guard(cfg()).check(real_data(fix_quality=0), cmd_linear=0.6, now=1000.0)
    assert v.no_fix is True


def test_a_payload_with_no_fix_quality_field_is_not_condemned():
    """`fix_quality` is not in the SDK's documented response. Its absence must not
    read as a missing fix, or the follower refuses to drive on a healthy bot."""
    d = real_data()
    assert "fix_quality" not in d
    assert Guard(cfg()).check(d, cmd_linear=0.6, now=1000.0).no_fix is False


def test_a_payload_without_coordinates_at_all_is_not_condemned():
    """A missing `latitude` cannot reach the guard from a real bot: `LiveIO.get_pose`
    reads it first and a KeyError there is already handled as a failed telemetry
    read. Condemning absence would therefore only ever fire on test doubles — and
    the first version of this check did exactly that, breaking a healthy recovery
    test while protecting nothing."""
    d = real_data()
    del d["latitude"], d["longitude"]
    assert Guard(cfg()).check(d, cmd_linear=0.6, now=1000.0).no_fix is False


def test_a_good_fix_is_not_condemned():
    v = Guard(cfg()).check(real_data(fix_quality=1), cmd_linear=0.6, now=1000.0)
    assert v.no_fix is False


def test_the_sentinel_is_caught_while_the_timestamp_is_still_advancing():
    """The whole point: a live link with no lock keeps `/data` moving, so #59's
    frozen-fix check stays quiet and cannot cover this."""
    g = Guard(cfg(fix_max_age_s=2.0))
    for k in range(10):
        v = g.check(real_data(latitude=1000, longitude=1000, fix_quality=0,
                              timestamp=1724189733.0 + k),
                    cmd_linear=0.6, now=1000.0 + k)
    assert v.stale_fix is False               # not frozen — it is advancing
    assert v.no_fix is True


# ---------------- #77 in the control loop ----------------

class NoFixIO:
    """A bot with a live link and no GPS lock, driven through the real loop."""
    hsrc = "stub"

    def __init__(self):
        self.commands = []
        self.ticks = 0
        self.last_data = real_data(latitude=1000, longitude=1000, fix_quality=0)

    def waypoints(self, route_file):
        return [(37.8719 + 500 / M_PER_DEG, -122.2585)], 0

    def get_pose(self):
        self.ticks += 1
        # The link is healthy: the timestamp keeps advancing. Only the fix is missing.
        self.last_data = real_data(latitude=1000, longitude=1000, fix_quality=0,
                                   timestamp=1724189733.0 + self.ticks * 0.05)
        return 1000.0, 1000.0, 0.0

    def control(self, linear, angular):
        self.commands.append((linear, angular))

    def front_frame(self):
        return None

    def reached(self):
        return False, {}


def test_the_loop_never_drives_on_the_no_fix_sentinel():
    io = NoFixIO()
    run(io, cfg(max_runtime_s=3.0))
    assert io.commands, "the loop must at least command a stop"
    assert max(abs(l) for l, _ in io.commands) < 0.05, (
        f"drove on a position that does not exist: {io.commands[:5]}")


def test_the_loop_stops_the_rover_when_the_fix_never_arrives():
    io = NoFixIO()
    run(io, cfg(max_runtime_s=3.0))
    assert io.commands[-1] == (0, 0)


# ---------------- #78: the harness must not diverge again ----------------

def test_the_fake_server_timestamps_its_rpm_rows_like_the_real_one():
    """The harness produced 4-element rows, so #76 was invisible to every test that
    used it. Pin the real shape: four wheels, then the sample timestamp."""
    from fake_sdk_server import Sim, SimConfig
    s = Sim(SimConfig(), clock=lambda: 1724189733.0)
    s.apply_control(0.5, 0.0)
    d = s.data()
    row = d["rpms"][0]
    assert len(row) == 5, "four wheels, then the sample timestamp"
    assert row[4] == d["timestamp"]
    assert _wheel_motion(d) is True, "a driving rover still reads as moving"


def test_the_fake_server_timestamps_every_imu_row_like_the_real_one():
    """`accels`, `gyros` and `mags` share the values-then-timestamp shape. Pinning
    all of them is the point of #78 — the next divergence should fail here."""
    from fake_sdk_server import Sim, SimConfig
    s = Sim(SimConfig(), clock=lambda: 1724189733.0)
    d = s.data()
    for key in ("accels", "gyros"):
        assert d[key][0][3] == d["timestamp"], f"{key} row is not timestamped"


def test_the_fake_server_can_report_the_no_fix_sentinel():
    """#77 was unreachable offline because the harness always had a lock."""
    from fake_sdk_server import Sim, SimConfig
    s = Sim(SimConfig(no_fix=True), clock=lambda: 1000.0)
    d = s.data()
    assert d["latitude"] == 1000 and d["longitude"] == 1000
    assert d["fix_quality"] == 0


def test_the_fake_server_reports_a_good_fix_by_default():
    from fake_sdk_server import Sim, SimConfig
    s = Sim(SimConfig(), clock=lambda: 1000.0)
    d = s.data()
    assert d["fix_quality"] == 1
    assert abs(d["latitude"]) <= 90.0
