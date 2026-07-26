"""Vision fusion hardening (issues #10, #11, #12).

The fusion was: blend GPS and vision steering 50/50, always, and take whichever
speed is lower. Three ways that goes wrong on a real bot:

  #10 `/v2/front` returns "the latest emitted frame" from a ~500 ms WebRTC stream.
      The timestamp was discarded, so a stalled stream steers the rover on a scene
      that no longer exists, and nothing notices.
  #11 The blend was applied unconditionally — including at a 150 deg heading error,
      where the GPS controller is deliberately turning in place and the policy has
      never seen anything like the view. No confidence signal either.
  #12 `min(gps_linear, vision_linear)` with a behaviour-cloned `linear` regressed
      on hesitant human teleop pins the rover to a crawl. Score is difficulty x
      time, so a crawl is nearly as bad as a failure.
"""
from waypoint_follower import Config, _fuse, fuse_gate, frame_is_fresh


def cfg(**kw):
    c = Config()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


# ---------------- #10: stale frames ----------------

def test_a_recent_frame_is_fresh():
    assert frame_is_fresh(timestamp=999.5, now=1000.0, cfg=cfg(frame_max_age_s=0.7)) is True


def test_an_old_frame_is_not_fresh():
    assert frame_is_fresh(timestamp=998.0, now=1000.0, cfg=cfg(frame_max_age_s=0.7)) is False


def test_a_frame_with_no_timestamp_is_accepted_but_only_because_we_cannot_tell():
    assert frame_is_fresh(timestamp=None, now=1000.0, cfg=cfg()) is True


def test_a_timestamp_in_milliseconds_is_understood():
    """The SDK's timestamps are unix seconds, but a float in the 1e12 range is
    obviously milliseconds — do not treat it as a 40000-year-old frame."""
    assert frame_is_fresh(timestamp=1000_000.0 * 1000, now=1000_000.0, cfg=cfg()) is True


# ---------------- #11: gating ----------------

def test_vision_is_used_when_roughly_aligned():
    assert fuse_gate(err_deg=10.0, confidence=1.0, cfg=cfg()) > 0


def test_vision_is_ignored_while_turning_in_place():
    """At 150 deg error the GPS controller is spinning to face the waypoint; the
    policy has no opinion worth having about that view."""
    assert fuse_gate(err_deg=150.0, confidence=1.0, cfg=cfg()) == 0.0


def test_low_confidence_falls_back_to_gps():
    assert fuse_gate(err_deg=0.0, confidence=0.0, cfg=cfg()) == 0.0


def test_confidence_scales_the_blend():
    strong = fuse_gate(err_deg=0.0, confidence=1.0, cfg=cfg())
    weak = fuse_gate(err_deg=0.0, confidence=0.4, cfg=cfg())
    assert 0.0 < weak < strong


def test_alpha_is_configurable():
    assert (fuse_gate(err_deg=0.0, confidence=1.0, cfg=cfg(vision_alpha=0.8))
            > fuse_gate(err_deg=0.0, confidence=1.0, cfg=cfg(vision_alpha=0.2)))


# ---------------- #12: speed floor ----------------

def test_a_timid_vision_policy_cannot_pin_the_rover_to_a_crawl():
    lin, _ = _fuse(gps_ang=0.0, vis_ang=0.0, gps_lin=0.6, vis_lin=0.02,
                   alpha=0.5, min_linear=0.25)
    assert lin >= 0.25


def test_the_gps_controller_can_still_stop_the_rover():
    """The floor must not override a deliberate stop — turning in place, or a
    checkpoint approach."""
    lin, _ = _fuse(gps_ang=0.5, vis_ang=0.0, gps_lin=0.0, vis_lin=0.6,
                   alpha=0.5, min_linear=0.25)
    assert lin == 0.0


def test_vision_still_slows_the_rover_within_the_allowed_range():
    slow, _ = _fuse(0.0, 0.0, 0.6, 0.3, alpha=0.5, min_linear=0.25)
    fast, _ = _fuse(0.0, 0.0, 0.6, 0.6, alpha=0.5, min_linear=0.25)
    assert slow < fast


def test_alpha_zero_means_pure_gps_steering():
    _, ang = _fuse(gps_ang=0.6, vis_ang=-0.6, gps_lin=0.6, vis_lin=0.6, alpha=0.0)
    assert ang == 0.6


def test_alpha_one_means_pure_vision_steering():
    _, ang = _fuse(gps_ang=0.6, vis_ang=-0.6, gps_lin=0.6, vis_lin=0.6, alpha=1.0)
    assert ang == -0.6
