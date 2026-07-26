"""Calibration must not leave the rover driving (issue #32).

calibrate_heading.py drives forward at CAL_THROTTLE for a few seconds with a human
standing next to it. The stop was after the loop, not in a finally — so a
get_data() failure (a normal 4G event, and the reason RoverClient retries at all)
or a Ctrl-C skipped it, and the rover kept its last command: forward.
"""
import pytest

from calibrate_heading import collect


class FakeClient:
    def __init__(self, fail_after=None, lat_step=0.0):
        self.commands = []
        self.calls = 0
        self.fail_after = fail_after
        self.lat = 37.8719
        self.lat_step = lat_step

    def get_data(self):
        self.calls += 1
        if self.fail_after is not None and self.calls > self.fail_after:
            raise RuntimeError("connection reset")
        self.lat += self.lat_step
        return {"latitude": self.lat, "longitude": -122.2585, "orientation": 64}

    def control(self, linear, angular):
        self.commands.append((linear, angular))


def test_a_clean_run_stops_the_rover():
    c = FakeClient(lat_step=1e-5)
    collect(c, secs=0.3, hz=20.0, throttle=0.5)
    assert c.commands[-1] == (0, 0)


def test_a_telemetry_failure_still_stops_the_rover():
    c = FakeClient(fail_after=3, lat_step=1e-5)
    with pytest.raises(RuntimeError):
        collect(c, secs=5.0, hz=20.0, throttle=0.5)
    assert c.commands[-1] == (0, 0), "left the rover driving after a failure"


def test_the_stop_is_retried_when_it_fails():
    class Stubborn(FakeClient):
        def __init__(self):
            super().__init__(lat_step=1e-5)
            self.stop_failures = 3

        def control(self, linear, angular):
            if linear == 0 and angular == 0 and self.stop_failures:
                self.stop_failures -= 1
                raise RuntimeError("send failed")
            super().control(linear, angular)

    c = Stubborn()
    collect(c, secs=0.2, hz=20.0, throttle=0.5, stop_attempts=8, stop_gap_s=0.0)
    assert (0, 0) in c.commands


def test_a_keyboard_interrupt_stops_the_rover():
    class Interrupting(FakeClient):
        def get_data(self):
            self.calls += 1
            if self.calls > 2:
                raise KeyboardInterrupt
            return super().get_data()

    c = Interrupting(lat_step=1e-5)
    with pytest.raises(KeyboardInterrupt):
        collect(c, secs=5.0, hz=20.0, throttle=0.5)
    assert c.commands[-1] == (0, 0)


def test_it_returns_the_samples_it_collected():
    c = FakeClient(lat_step=1e-5)
    samples = collect(c, secs=0.3, hz=20.0, throttle=0.5)
    assert len(samples) >= 3
    assert all(len(s) == 3 for s in samples)
