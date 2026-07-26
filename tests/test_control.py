import pytest

from waypoint_follower import Config, steer, _fuse

CFG = Config()


def test_steer_aligned_cruises():
    lin, ang, err = steer(100, 0, 0, CFG)
    assert abs(err) < 1
    assert abs(ang) < 1e-6                    # deadband -> no steering
    assert abs(lin - CFG.cruise) < 1e-6       # full cruise far from wp


def test_steer_turn_right():
    lin, ang, err = steer(100, 90, 0, CFG)    # target to the east, facing north
    assert ang > 0.5                          # steer right
    assert err > 0


def test_steer_target_behind_no_forward():
    lin, ang, err = steer(100, 180, 0, CFG)   # target directly behind
    assert lin == 0.0                         # don't drive forward while flipping around


def test_steer_approach_slowdown():
    far = steer(100, 0, 0, CFG)[0]
    near = steer(1.0, 0, 0, CFG)[0]
    assert near < far                         # creeps near the checkpoint


def test_fuse_agree_reinforces():
    lin, ang = _fuse(0.6, 0.6, 0.6, 0.6)
    assert ang > 0.5


def test_fuse_conflict_cancels():
    lin, ang = _fuse(0.6, -0.6, 0.6, 0.6)
    assert abs(ang) < 1e-6


# Heading estimation moved to heading.py — see tests/test_heading.py. The old test
# here asserted that a 2 m move yields a trusted GPS course, which is the bug in #1:
# under real GPS noise a 2 m "move" is indistinguishable from standing still.


# ---------------- the bearing is noise at close range (issue #66) ----------------

def test_steering_is_damped_when_the_target_is_metres_away():
    """With sigma=1.5 m GPS the bearing standard deviation is 2.5 deg at 30 m, 14 deg
    at 5 m and 55 deg at 2 m. Steering hard on a 55 deg 'error' that is actually noise
    makes the rover spin on the spot, which costs time — and the challenge scores
    difficulty x time."""
    cfg = Config()
    far_lin, far_ang, _ = steer(30.0, 55.0, 0.0, cfg)
    near_lin, near_ang, _ = steer(1.5, 55.0, 0.0, cfg)
    assert abs(near_ang) < abs(far_ang) * 0.5, (
        f"same 55 deg error: {far_ang:.2f} at 30 m vs {near_ang:.2f} at 1.5 m")


def test_close_range_damping_does_not_stop_us_turning_at_all():
    """Damped, not disabled — the rover still needs to point roughly the right way."""
    lin, ang, _ = steer(1.5, 90.0, 0.0, Config())
    assert abs(ang) > 0.0


def test_normal_range_steering_is_unchanged():
    cfg = Config()
    for dist in (10.0, 30.0, 100.0):
        assert steer(dist, 45.0, 0.0, cfg)[1] == pytest.approx(1.0, abs=0.01)


def test_the_damping_distance_is_configurable():
    cfg = Config()
    cfg.bearing_trust_m = 0.0                     # disabled
    assert steer(1.0, 55.0, 0.0, cfg)[1] == pytest.approx(
        steer(30.0, 55.0, 0.0, cfg)[1], abs=0.01)
