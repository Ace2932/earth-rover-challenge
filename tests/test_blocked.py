"""Stopping for what is in front of the rover (issue #7).

The Urban track is sidewalks, crossings and pedestrians. The policy regressed
steering only: it had no way to say "do not go forward", so nothing in the stack
could stop for anything.

Two halves, tested separately:
  * `BlockedGate` — the decision to stop, with hysteresis, so a single noisy frame
    cannot brake the rover and a single confident frame cannot un-brake it.
  * `blocked_labels` — turning teleop into a weak "the human stopped here" signal.
"""
import pytest

from blocked import BlockedGate, blocked_labels
from waypoint_follower import Config


def cfg(**kw):
    c = Config()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


# ---------------- the stop decision ----------------

def test_a_clear_path_does_not_stop_the_rover():
    g = BlockedGate(cfg(blocked_p=0.7, blocked_frames=2))
    assert g.update(0.1, now=1000.0) is False
    assert g.update(0.2, now=1000.2) is False


def test_one_alarming_frame_is_not_enough_to_brake():
    """A single frame of a passing shadow should not stop a mission."""
    g = BlockedGate(cfg(blocked_p=0.7, blocked_frames=3))
    assert g.update(0.99, now=1000.0) is False


def test_consecutive_alarming_frames_stop_the_rover():
    g = BlockedGate(cfg(blocked_p=0.7, blocked_frames=3))
    g.update(0.9, now=1000.0)
    g.update(0.9, now=1000.2)
    assert g.update(0.9, now=1000.4) is True


def test_a_clear_frame_resets_the_count():
    g = BlockedGate(cfg(blocked_p=0.7, blocked_frames=3))
    g.update(0.9, now=1000.0)
    g.update(0.1, now=1000.2)
    g.update(0.9, now=1000.4)
    assert g.update(0.9, now=1000.6) is False      # only 2 in a row


def test_the_stop_is_held_briefly_so_it_cannot_chatter():
    """Stop/go/stop at 5 Hz is worse for a pedestrian than a clean stop."""
    g = BlockedGate(cfg(blocked_p=0.7, blocked_frames=1, blocked_hold_s=1.0))
    assert g.update(0.9, now=1000.0) is True
    assert g.update(0.0, now=1000.5) is True       # still held
    assert g.update(0.0, now=1001.5) is False      # released


def test_a_missing_probability_never_stops_the_rover():
    """A model without the head, or a failed inference, must not brake the rover."""
    g = BlockedGate(cfg())
    for t in range(10):
        assert g.update(None, now=1000.0 + t) is False


def test_the_gate_is_disabled_when_the_threshold_is_one():
    g = BlockedGate(cfg(blocked_p=1.0, blocked_frames=1))
    assert g.update(0.999, now=1000.0) is False


# ---------------- the weak label ----------------

def test_a_braking_event_is_labelled_blocked():
    """Moving, then stopped: something made the human stop."""
    linear = [0.6] * 10 + [0.0] * 10
    labels = blocked_labels(linear, hz=10.0, moving=0.3, stopped=0.05, lookback_s=1.0)
    assert labels[0] == 0
    assert labels[12] == 1


def test_a_rover_that_was_never_moving_is_not_blocked():
    """Parked at the start of a ride is not an obstacle."""
    labels = blocked_labels([0.0] * 20, hz=10.0, moving=0.3, stopped=0.05, lookback_s=1.0)
    assert set(labels) == {0}


def test_steady_driving_is_not_blocked():
    labels = blocked_labels([0.6] * 20, hz=10.0, moving=0.3, stopped=0.05, lookback_s=1.0)
    assert set(labels) == {0}


def test_a_stop_long_after_moving_is_not_attributed_to_an_obstacle():
    """Ten seconds parked is waiting, not braking."""
    linear = [0.6] * 5 + [0.0] * 100
    labels = blocked_labels(linear, hz=10.0, moving=0.3, stopped=0.05, lookback_s=1.0)
    assert labels[8] == 1          # just after the stop
    assert labels[90] == 0         # long after


def test_slowing_without_stopping_is_not_blocked():
    linear = [0.6] * 10 + [0.2] * 10
    labels = blocked_labels(linear, hz=10.0, moving=0.3, stopped=0.05, lookback_s=1.0)
    assert set(labels) == {0}


# ---------------- the policy head ----------------

def test_the_policy_can_report_a_blocked_probability():
    torch = pytest.importorskip("torch")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "vision"))
    from policy import SidewalkPolicy

    model = SidewalkPolicy(backbone="tiny", blocked_head=True)
    lin, ang, p = model.act(torch.zeros(3, 64, 64))
    assert 0.0 <= p <= 1.0
    assert -1.0 <= ang <= 1.0


def test_a_policy_without_the_head_reports_no_opinion():
    """Old two-output checkpoints must keep loading, and must not fake a stop signal."""
    torch = pytest.importorskip("torch")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "vision"))
    from policy import SidewalkPolicy

    model = SidewalkPolicy(backbone="tiny")
    lin, ang, p = model.act(torch.zeros(3, 64, 64))
    assert p is None
