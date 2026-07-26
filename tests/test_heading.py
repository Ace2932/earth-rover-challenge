"""Heading estimator tests (issue #1).

The old estimator took a GPS course over a 0.7 m baseline. Real GPS noise is
metres, so that course was noise, not motion — and because noise alone moves a
parked rover "0.7 m", it claimed a GPS heading while standing still.

These tests pin the properties that make a course trustworthy: the rover must
actually be moving, the baseline must be long, the rover must not have turned
much while covering it, and the result must be filtered rather than believed
outright.
"""
import math
import random

from geo import wrap180
from heading import HeadingEstimator
from waypoint_follower import Config

M_PER_DEG = 111111.0


def cfg(**kw):
    c = Config()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


class Track:
    """Straight-line ground truth with iid GPS noise, sampled at `hz`."""

    def __init__(self, heading_deg=0.0, speed=0.9, sigma=1.5, hz=5.0, seed=0,
                 lat=37.8719, lon=-122.2585):
        self.h = heading_deg
        self.speed = speed
        self.sigma = sigma
        self.dt = 1.0 / hz
        self.rng = random.Random(seed)
        self.lat, self.lon = lat, lon
        self.t = 1000.0

    def step(self, moving=True):
        if moving:
            d = self.speed * self.dt
            self.lat += d * math.cos(math.radians(self.h)) / M_PER_DEG
            self.lon += (d * math.sin(math.radians(self.h))
                         / (M_PER_DEG * math.cos(math.radians(self.lat))))
        self.t += self.dt
        return (self.lat + self.rng.gauss(0, self.sigma) / M_PER_DEG,
                self.lon + self.rng.gauss(0, self.sigma) / M_PER_DEG,
                self.t)


def test_first_update_falls_back_to_the_magnetometer():
    est = HeadingEstimator(cfg(heading_scale=360.0 / 255.0))
    h, src = est.update(37.8719, -122.2585, orientation=64, now=1000.0)
    assert src == "mag"
    assert h == (64 * 360.0 / 255.0) % 360.0


def test_magnetometer_mapping_honours_scale_offset_and_sign():
    est = HeadingEstimator(cfg(heading_scale=2.0, heading_offset=30.0, heading_sign=-1.0))
    h, _ = est.update(37.8719, -122.2585, orientation=45, now=1000.0)
    assert h == (-1.0 * 45 * 2.0 + 30.0) % 360.0


def test_parked_rover_never_produces_a_gps_course():
    """The old estimator's worst failure: GPS noise alone crosses the movement
    threshold, so a stationary rover gets a random heading ~90% of the time."""
    est = HeadingEstimator(cfg())
    trk = Track(sigma=2.0, seed=1)
    sources = []
    for _ in range(300):
        lat, lon, t = trk.step(moving=False)
        sources.append(est.update(lat, lon, orientation=0, now=t,
                                  cmd_linear=0.0, cmd_angular=0.0)[1])
    assert "gps" not in sources


def test_commanded_motion_alone_is_not_enough_when_telemetry_says_stopped():
    """Commanded forward, wheels not turning (high-centred, or the command was
    dropped over RTM) -> the displacement is noise, so reject it."""
    est = HeadingEstimator(cfg())
    trk = Track(sigma=2.0, seed=2)
    sources = []
    for _ in range(300):
        lat, lon, t = trk.step(moving=False)
        sources.append(est.update(lat, lon, orientation=0, now=t,
                                  cmd_linear=0.6, speed=0.0)[1])
    assert "gps" not in sources


def test_baseline_is_measured_by_odometry_not_by_noisy_gps():
    """Gating on the GPS-measured displacement selects for exactly the samples where
    noise inflated the baseline — the ones whose direction is least reliable. Here the
    rover crawls 2 m in 10 s while sigma=3 m noise regularly "moves" it much further."""
    est = HeadingEstimator(cfg())
    trk = Track(speed=0.2, sigma=3.0, seed=3)
    sources = []
    for _ in range(50):
        lat, lon, t = trk.step()
        sources.append(est.update(lat, lon, orientation=0, now=t,
                                  cmd_linear=0.2, speed=0.2)[1])
    assert "gps" not in sources


def test_course_is_rejected_when_the_rover_turned_while_covering_the_baseline():
    """A chord across a curve is not a heading."""
    est = HeadingEstimator(cfg(heading_min_move_m=6.0, heading_max_turn_deg=30.0))
    trk = Track(sigma=0.0, seed=4)
    sources = []
    for _ in range(100):
        trk.h = (trk.h + 9.0) % 360.0                  # 45 deg/s at 5 Hz
        lat, lon, t = trk.step()
        sources.append(est.update(lat, lon, orientation=0, now=t,
                                  cmd_linear=0.6, cmd_angular=0.5, speed=0.9)[1])
    assert "gps" not in sources


def test_gps_course_corrects_a_badly_wrong_magnetometer():
    est = HeadingEstimator(cfg())
    trk = Track(heading_deg=0.0, sigma=0.3, seed=5)
    h = None
    for _ in range(400):                                # 80 s, ~72 m of travel
        lat, lon, t = trk.step()
        h, _ = est.update(lat, lon, orientation=64, now=t,   # mag insists ~90 deg
                          cmd_linear=0.6, speed=0.9)
    assert abs(wrap180(h - 0.0)) < 15.0


def _run_track(sigma, seed, secs=120.0, hz=5.0, mag_deg=90.0):
    """Drive due north with a magnetometer that insists we are heading east."""
    est = HeadingEstimator(cfg())
    trk = Track(heading_deg=0.0, sigma=sigma, hz=hz, seed=seed)
    errs = []
    for i in range(int(secs * hz)):
        lat, lon, t = trk.step()
        h, _ = est.update(lat, lon, orientation=mag_deg / (360.0 / 255.0), now=t,
                          cmd_linear=0.6, speed=trk.speed)
        if i > 0.4 * secs * hz:                          # after lock-on
            errs.append(abs(wrap180(h - 0.0)))
    errs.sort()
    return errs[len(errs) // 2], errs[int(0.9 * len(errs))]


def test_median_error_under_realistic_noise_is_within_12_deg():
    """Issue #1's acceptance criterion, over several noise realisations so it cannot
    pass by seed luck. Same scenario as the review's Monte Carlo: 0.9 m/s, 5 Hz,
    sigma=1.5 m, 120 s, magnetometer 90 deg wrong. Old estimator: ~90 deg median."""
    for seed in (1, 2, 3, 4, 5):
        median, p90 = _run_track(sigma=1.5, seed=seed)
        assert median < 12.0, f"seed {seed}: median {median:.1f} deg"
        assert p90 < 25.0, f"seed {seed}: p90 {p90:.1f} deg"


def test_degrades_gracefully_at_3m_gps_noise():
    """Urban canyon. The old estimator was ~90 deg here — no better than useless."""
    for seed in (1, 2, 3):
        median, _ = _run_track(sigma=3.0, seed=seed)
        assert median < 25.0, f"seed {seed}: median {median:.1f} deg"


def test_wheel_slip_is_rejected_when_gps_disagrees_with_odometry():
    """Odometry claims 8 m of travel, GPS says we barely moved: wheels spinning, or
    the rover is high-centred. Separable at low noise; beyond ~2 m of GPS noise the
    two distributions overlap and slip is simply not detectable from GPS alone —
    a limit of GPS, not of this check. Stuck detection (#4) is the backstop there."""
    est = HeadingEstimator(cfg())
    trk = Track(sigma=1.0, seed=9)
    sources = []
    for _ in range(300):
        lat, lon, t = trk.step(moving=False)             # not actually moving
        sources.append(est.update(lat, lon, orientation=0, now=t,
                                  cmd_linear=0.6, speed=0.9)[1])
    assert "gps" not in sources


def test_dead_reckoning_advances_heading_with_the_commanded_yaw():
    est = HeadingEstimator(cfg(yaw_rate_dps=90.0))
    est.update(37.8719, -122.2585, orientation=0, now=1000.0)      # seed from mag: 0 deg
    h, src = est.update(37.8719, -122.2585, orientation=0, now=1001.0, cmd_angular=0.5)
    assert src == "dr"
    assert abs(wrap180(h - 45.0)) < 1.0


def test_gyro_yaw_rate_overrides_the_commanded_estimate():
    est = HeadingEstimator(cfg(yaw_rate_dps=90.0))
    est.update(37.8719, -122.2585, orientation=0, now=1000.0)
    h, src = est.update(37.8719, -122.2585, orientation=0, now=1001.0,
                        cmd_angular=0.5, gyro_z_dps=-30.0)
    assert src == "dr"
    assert abs(wrap180(h - (-30.0))) < 1.0


def test_heading_stays_in_0_360():
    est = HeadingEstimator(cfg(yaw_rate_dps=90.0))
    est.update(37.8719, -122.2585, orientation=0, now=1000.0)
    for i in range(20):
        h, _ = est.update(37.8719, -122.2585, orientation=0, now=1001.0 + i,
                          cmd_angular=-1.0)
        assert 0.0 <= h < 360.0


# ---------------- corrections while turning (issue #44) ----------------

def drive_arc(est, turn_dps, secs=300.0, hz=5.0, sigma=0.0, speed=0.9, seed=0):
    """Drive a constant-rate arc; return (gps fixes, final |heading error|)."""
    rng = random.Random(seed)
    lat, lon, t, truth = 37.8719, -122.2585, 1000.0, 0.0
    fixes, h = 0, 0.0
    for _ in range(int(secs * hz)):
        truth = (truth + turn_dps / hz) % 360.0
        d = speed / hz
        lat += d * math.cos(math.radians(truth)) / M_PER_DEG
        lon += d * math.sin(math.radians(truth)) / (M_PER_DEG * math.cos(math.radians(lat)))
        t += 1.0 / hz
        h, src = est.update(lat + rng.gauss(0, sigma) / M_PER_DEG,
                            lon + rng.gauss(0, sigma) / M_PER_DEG,
                            orientation=0, now=t, cmd_linear=0.6,
                            cmd_angular=turn_dps / est.cfg.yaw_rate_dps, speed=speed)
        fixes += src == "gps"
    return fixes, abs(wrap180(h - truth))


def test_a_gentle_curve_still_gets_gps_corrections():
    """3.4 deg/s was the old ceiling: above it the turn gate rejected every sample
    and the filter went permanently blind. A sidewalk course is mostly curves."""
    fixes, _ = drive_arc(HeadingEstimator(cfg()), turn_dps=5.0)
    assert fixes > 0


def test_an_orbit_gets_gps_corrections():
    """The CI trace: 12 deg/s, circling a checkpoint, never correcting."""
    fixes, _ = drive_arc(HeadingEstimator(cfg()), turn_dps=12.0)
    assert fixes > 0


def test_heading_stays_accurate_around_a_curve():
    """A chord across a turn is not the end heading — it is the AVERAGE heading, so
    it needs de-biasing by half the turn before it is used as a correction."""
    _, err = drive_arc(HeadingEstimator(cfg()), turn_dps=8.0, secs=200.0)
    assert err < 20.0, f"heading drifted to {err:.1f} deg around a curve"


def test_a_hard_turn_is_still_rejected():
    """Past a point the chord really is meaningless — do not correct from it."""
    est = HeadingEstimator(cfg())
    fixes, _ = drive_arc(est, turn_dps=60.0, secs=120.0)
    assert fixes == 0


def test_the_filter_cannot_go_blind_indefinitely():
    """Belt and braces: if nothing has been accepted for a long time while moving,
    take the next sample anyway rather than dead-reckoning forever."""
    c = cfg(heading_max_turn_deg=0.0)          # reject everything on turn grounds
    est = HeadingEstimator(c)
    fixes, _ = drive_arc(est, turn_dps=2.0, secs=200.0)
    assert fixes > 0


# ---------------- wobble must not starve the filter (issue #54) ----------------

def drive_wobble(wobble_dps, sigma=1.5, secs=200.0, hz=5.0, seed=3):
    """Straight line, but the controller oscillates chasing a noisy bearing.
    Net turn is ~0: the rover is going straight."""
    c = cfg()
    est = HeadingEstimator(c)
    rng = random.Random(seed)
    lat, lon, t, truth = 37.8719, -122.2585, 1000.0, 0.0
    fixes = 0
    for k in range(int(secs * hz)):
        yaw = wobble_dps * (1 if (k // 3) % 2 == 0 else -1)
        truth = (truth + yaw / hz) % 360.0
        d = 0.9 / hz
        lat += d * math.cos(math.radians(truth)) / M_PER_DEG
        lon += d * math.sin(math.radians(truth)) / (M_PER_DEG * math.cos(math.radians(lat)))
        t += 1.0 / hz
        _, src = est.update(lat + rng.gauss(0, sigma) / M_PER_DEG,
                            lon + rng.gauss(0, sigma) / M_PER_DEG,
                            orientation=0, now=t, cmd_linear=0.6,
                            cmd_angular=yaw / c.yaw_rate_dps, speed=0.9)
        fixes += src == "gps"
    return fixes


def test_a_wobbling_controller_still_gets_corrections():
    """Noisy GPS makes the bearing jitter, so the controller wobbles — and summing
    ABSOLUTE yaw counts that as turning. At +/-10 deg/s the accumulated |yaw| over
    one 8.9 s baseline is 89 deg, right at the gate, while net turn is ~0 and the
    rover is going straight."""
    steady = drive_wobble(0)
    for wobble in (10, 20):
        got = drive_wobble(wobble)
        assert got > 0.6 * steady, (
            f"wobbling +/-{wobble} deg/s dropped corrections to {got} of {steady}")


def test_a_sustained_net_turn_is_still_rejected():
    """Net rotation is what the chord cannot describe, and it must still be caught."""
    fixes, _ = drive_arc(HeadingEstimator(cfg()), turn_dps=60.0, secs=120.0)
    assert fixes == 0


# ---------------- multipath jumps (issue #61) ----------------

def drive_with_jump(jump_m, jump_step=45, steps=60, hz=5.0):
    """Straight north, with one lateral multipath jump on a single sample."""
    est = HeadingEstimator(cfg())
    lat, lon, t = 37.8719, -122.2585, 1000.0
    worst = 0.0
    for k in range(steps):
        lat += (0.9 / hz) / M_PER_DEG
        t += 1.0 / hz
        j = jump_m if k == jump_step else 0.0
        h, _ = est.update(lat, lon + (j / M_PER_DEG) / math.cos(math.radians(lat)),
                          orientation=0, now=t, cmd_linear=0.6, speed=0.9)
        worst = max(worst, abs(wrap180(h)))
    return worst


def test_a_multipath_jump_cannot_swing_the_heading():
    """A jump landing on the sample that completes the odometry baseline produces a
    chord pointing somewhere arbitrary. The slip check only rejects chords that are
    too SHORT; a jump makes them too LONG, and the rover cannot have travelled
    further than its wheels turned."""
    assert drive_with_jump(10.0) < 15.0
    assert drive_with_jump(30.0) < 15.0


def test_a_normal_course_is_still_accepted():
    """The upper bound must not reject honest chords — odometry under-reports a
    little, so there has to be headroom."""
    est = HeadingEstimator(cfg())
    trk = Track(heading_deg=0.0, sigma=0.5, seed=4)
    fixes = 0
    for _ in range(400):
        lat, lon, t = trk.step()
        _, src = est.update(lat, lon, orientation=0, now=t, cmd_linear=0.6, speed=0.9)
        fixes += src == "gps"
    assert fixes > 5
