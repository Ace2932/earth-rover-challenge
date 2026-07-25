"""Reading the rest of `/data` and acting on it.

The follower used three fields — latitude, longitude, orientation — out of a
payload that also carries battery, signal_level, gps_signal, speed and rpms. So
it would happily drive at full cruise on a dying battery, trust a fix in an urban
canyon as much as one in open sky, and never notice that a command produced no
wheel motion at all.

`Guard` is pure: telemetry in, a verdict out. No I/O, no clock of its own.
"""
from dataclasses import dataclass, field


@dataclass
class Verdict:
    speed_scale: float = 1.0        # multiply the commanded throttle by this
    abort: str = None               # non-None -> stop the run, with this reason
    no_motion: bool = False         # commanded to move, wheels say otherwise
    warnings: list = field(default_factory=list)


def _num(data, key):
    try:
        v = data[key]
        return None if v is None else float(v)
    except (KeyError, TypeError, ValueError):
        return None


def _wheel_motion(data):
    """Any evidence of movement: reported speed, or a turning wheel."""
    speed = _num(data, "speed")
    if speed is not None and abs(speed) > 0.05:
        return True
    try:
        return any(abs(float(r)) > 1.0 for sample in data.get("rpms") or []
                   for r in (sample if isinstance(sample, (list, tuple)) else [sample]))
    except (TypeError, ValueError):
        return False


class Guard:
    def __init__(self, cfg):
        self.cfg = cfg
        self.moving_since = None     # last time we saw real motion
        self.warned = set()

    def _once(self, key, message, verdict):
        if key not in self.warned:
            self.warned.add(key)
            verdict.warnings.append(message)

    def check(self, data, cmd_linear, now):
        v = Verdict()
        if not isinstance(data, dict):
            return v
        c = self.cfg

        battery = _num(data, "battery")
        if battery is not None:
            if battery <= c.battery_abort_pct:
                v.abort = (f"battery {battery:.0f}% is at or below the "
                           f"{c.battery_abort_pct:.0f}% floor")
                return v
            if battery <= c.battery_warn_pct:
                self._once("battery", f"battery down to {battery:.0f}%", v)

        if _num(data, "signal_level") == 0:
            self._once("signal", "4G signal_level is 0 — expect dropped commands", v)

        # gps_signal's units are not documented. Treat it as "higher is better"
        # between two configured thresholds, and allow good == poor to disable the
        # whole thing rather than forcing a code change to escape a wrong guess.
        gps = _num(data, "gps_signal")
        if gps is not None and c.gps_signal_good > c.gps_signal_poor:
            if gps >= c.gps_signal_good:
                v.speed_scale = 1.0
            elif gps <= c.gps_signal_poor:
                v.speed_scale = c.min_speed_scale
                self._once("gps", f"gps_signal {gps:.0f} is poor — slowing down", v)
            else:
                span = c.gps_signal_good - c.gps_signal_poor
                frac = (gps - c.gps_signal_poor) / span
                v.speed_scale = c.min_speed_scale + frac * (1.0 - c.min_speed_scale)

        if abs(cmd_linear) < 0.05:
            self.moving_since = now          # not asked to move; nothing to check
        elif _wheel_motion(data):
            self.moving_since = now
        else:
            if self.moving_since is None:
                self.moving_since = now
            elif now - self.moving_since > c.no_motion_s:
                v.no_motion = True
        return v
