from waypoint_follower import Config, steer, _fuse, HeadingEstimator

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


def test_heading_estimator_gps_when_moving():
    est = HeadingEstimator(Config())
    h0, src0 = est.estimate(0.0, 0.0, 0.0)
    assert src0 == "mag"                      # first fix -> magnetometer
    north = 2.0 / 111111.0                    # ~2 m north (> heading_min_move_m)
    h1, src1 = est.estimate(north, 0.0, 0.0)
    assert src1 == "gps"
    assert abs(h1 - 0) < 1.0                  # course is due north
