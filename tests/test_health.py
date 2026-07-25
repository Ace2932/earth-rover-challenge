"""Health of the SDK/Chrome layer (issue #16).

The control chain runs through a headful Chrome page holding the Agora session.
If that page wedges, `/control` keeps returning 200 while nothing reaches the bot
— the failure is silent from the client's side. The one observable is `/data`'s
own `timestamp`: a live page advances it, a wedged one repeats it.
"""
from health import StaleDetector


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def det(clock, stale_s=10.0, cooldown_s=60.0):
    restarts = []
    d = StaleDetector(stale_s=stale_s, cooldown_s=cooldown_s, clock=clock,
                      restart=lambda: restarts.append(clock()))
    return d, restarts


def test_advancing_telemetry_is_healthy():
    clock = Clock()
    d, restarts = det(clock)
    for i in range(10):
        clock.t += 1.0
        assert d.observe({"timestamp": 1000.0 + i}) is True
    assert restarts == []


def test_a_repeated_timestamp_eventually_reads_as_wedged():
    """Chrome still serving, but the page stopped producing new telemetry."""
    clock = Clock()
    d, restarts = det(clock, stale_s=10.0)
    d.observe({"timestamp": 1000.0})
    clock.t += 30.0
    assert d.observe({"timestamp": 1000.0}) is False
    assert len(restarts) == 1


def test_a_brief_gap_is_not_a_restart():
    clock = Clock()
    d, restarts = det(clock, stale_s=10.0)
    d.observe({"timestamp": 1000.0})
    clock.t += 3.0
    assert d.observe({"timestamp": 1000.0}) is True
    assert restarts == []


def test_recovery_clears_the_stale_state():
    clock = Clock()
    d, restarts = det(clock, stale_s=10.0, cooldown_s=0.0)
    d.observe({"timestamp": 1000.0})
    clock.t += 30.0
    d.observe({"timestamp": 1000.0})
    clock.t += 1.0
    assert d.observe({"timestamp": 1031.0}) is True
    clock.t += 30.0
    assert d.observe({"timestamp": 1031.0}) is False
    assert len(restarts) == 2


def test_restarts_are_rate_limited_by_the_cooldown():
    """Restarting Chrome on every poll while it is coming back up helps nobody.
    Ten stale observations over 300 s must not mean ten restarts — they must be
    at least `cooldown_s` apart."""
    clock = Clock()
    d, restarts = det(clock, stale_s=10.0, cooldown_s=120.0)
    d.observe({"timestamp": 1000.0})
    for _ in range(10):
        clock.t += 30.0
        d.observe({"timestamp": 1000.0})
    assert 0 < len(restarts) <= 3
    assert all(b - a >= 120.0 for a, b in zip(restarts, restarts[1:]))


def test_a_missing_timestamp_falls_back_to_wall_clock_arrival():
    """No timestamp field: we can still tell whether responses are arriving."""
    clock = Clock()
    d, restarts = det(clock, stale_s=10.0)
    assert d.observe({}) is True
    clock.t += 30.0
    assert d.observe({}) is True          # a response IS the liveness signal here
    assert restarts == []


def test_a_failed_request_is_treated_as_no_telemetry():
    clock = Clock()
    d, restarts = det(clock, stale_s=10.0)
    d.observe({"timestamp": 1000.0})
    clock.t += 30.0
    assert d.observe(None) is False
    assert len(restarts) == 1
