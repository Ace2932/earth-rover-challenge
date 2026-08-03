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
    stale_fix: bool = False         # /data stopped updating; the position is fiction
    no_fix: bool = False            # /data is live but has no GPS lock; same result
    warnings: list = field(default_factory=list)


def _num(data, key):
    try:
        v = data[key]
        return None if v is None else float(v)
    except (KeyError, TypeError, ValueError):
        return None


WHEELS = 4      # the Mini+ has four; anything after them in the row is not a wheel


def _wheel_motion(data):
    """Any evidence of movement: reported speed, or a turning wheel.

    `/data`'s `rpms` rows are `[w1, w2, w3, w4, timestamp]` — the same
    values-then-timestamp shape `accels`, `gyros` and `mags` all use. Reading the
    whole row made every sample look like a wheel spinning at ~1.7e9 RPM, so this
    could not return False on a real bot and the no-motion check was dead (#76).

    Take the first four rather than dropping the last: a producer that omits the
    timestamp would otherwise silently lose a wheel, which is the same class of
    mistake in the other direction.
    """
    speed = _num(data, "speed")
    if speed is not None and abs(speed) > 0.05:
        return True
    try:
        return any(abs(float(r)) > 1.0 for sample in data.get("rpms") or []
                   for r in (sample[:WHEELS] if isinstance(sample, (list, tuple))
                             else [sample]))
    except (TypeError, ValueError):
        return False


class Guard:
    def __init__(self, cfg):
        self.cfg = cfg
        self.moving_since = None     # last time we saw real motion
        self.last_ts = None          # last /data timestamp we saw
        self.last_ts_change = None   # when it last changed, by OUR clock
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

        # Is there a fix at all? With no GPS lock the bot reports latitude and
        # longitude of 1000 and `fix_quality` 0 — observed on the bench 2026-07-30 —
        # while /data keeps ticking. So the freshness check below stays perfectly
        # quiet, and every bearing computed from that position is fiction: measured
        # at a 13 267 km "distance" driven at full cruise for a whole STUCK_S (#77).
        #
        # `fix_quality` is not in the SDK's documented response, so its ABSENCE must
        # never condemn a healthy bot; only an explicit 0 does. The coordinate range
        # check needs no undocumented field and catches the sentinel on its own.
        if _num(data, "fix_quality") == 0:
            v.no_fix = True
            self._once("nofix", "GPS reports no fix (fix_quality 0)", v)
        #
        # Only values that are PRESENT and impossible condemn the payload. A missing
        # latitude cannot reach here from a real bot — `LiveIO.get_pose` reads it
        # before the guard runs and a KeyError there is already handled as a failed
        # telemetry read — so treating absence as "no fix" would only ever fire on
        # test doubles, which is how the first version of this broke a healthy
        # recovery test while protecting nothing.
        lat, lon = _num(data, "latitude"), _num(data, "longitude")
        if (lat is not None and abs(lat) > 90.0) or (lon is not None and abs(lon) > 180.0):
            v.no_fix = True
            self._once("nofix_range", f"position {lat}, {lon} is not on Earth", v)

        # Is the position fix still alive? The SDK serves /data from a value cached
        # in the browser page and updated by incoming RTM messages, so a stalled link
        # keeps returning 200 with the last payload — position, speed and all. Judge
        # it by ADVANCEMENT rather than against our own clock: the bot's clock, the
        # SDK host's and ours need not agree, but a live fix keeps moving.
        ts = _num(data, "timestamp")
        if ts is None:
            self.last_ts, self.last_ts_change = None, None   # cannot tell; never stale
        else:
            if ts != self.last_ts:
                self.last_ts, self.last_ts_change = ts, now
            elif now - self.last_ts_change > c.fix_max_age_s:
                v.stale_fix = True
                self._once("fix", "position fix has stopped updating", v)

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
