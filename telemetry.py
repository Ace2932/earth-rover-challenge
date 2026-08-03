"""Reading the rest of `/data` and acting on it.

The follower used three fields — latitude, longitude, orientation — out of a
payload that also carries battery, signal_level, gps_signal, speed and rpms. So
it would happily drive at full cruise on a dying battery, trust a fix in an urban
canyon as much as one in open sky, and never notice that a command produced no
wheel motion at all.

`Guard` is pure: telemetry in, a verdict out. No I/O, no clock of its own.
"""
import os
from dataclasses import dataclass, field

from envcfg import FALSEY


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


def ignore_fix_quality_from_env():
    """IGNORE_FIX_QUALITY, for the tools that have no Config (#86).

    Named IGNORE_* rather than TRUST_* deliberately. `.env` placeholders in this repo
    are written `KEY=`, and `envcfg.FALSEY` treats an empty string as false — so with
    a TRUST_* flag, blanking the line would silently REMOVE the protection. Blanking
    this one leaves the guard on.
    """
    return (os.getenv("IGNORE_FIX_QUALITY") or "").strip().lower() not in FALSEY


def no_fix_reason(data, ignore_fix_quality=False):
    """Why this payload's position cannot be steered on, or None if it can.

    With no GPS lock the bot reports latitude and longitude of 1000 and
    `fix_quality` 0 — observed on the bench 2026-07-30 — while /data keeps ticking.
    So a freshness check stays perfectly quiet, and every bearing computed from that
    position is fiction: measured at a 13 267 km "distance" driven at full cruise
    for a whole STUCK_S (#77).

    `fix_quality` is not in the SDK's documented response, so its ABSENCE must never
    condemn a healthy bot; only an explicit 0 does. Likewise only coordinates that
    are PRESENT and impossible condemn a payload — a missing latitude cannot reach
    here from a real bot, because `LiveIO.get_pose` reads it first and a KeyError
    there is already handled as a failed telemetry read. Treating absence as "no
    fix" would fire on test doubles alone, which is how the first version of this
    broke a healthy recovery test while protecting nothing.

    The two halves are NOT equally certain, and only one of them is overridable:

    - The coordinate range check CANNOT be wrong. No real position has |lat| > 90.
      It catches the 1000/1000 sentinel on its own, needs no undocumented field, and
      stays unconditional.
    - `fix_quality` is the ASSUMPTION. It is undocumented and has been seen exactly
      once — indoors, alongside the sentinel, so the two were perfectly correlated
      and the sentinel alone explains what was observed. It has never been seen on a
      bot with a real lock. If "NMEA 0 = invalid" does not hold for this bot, the
      rover cannot be driven at all, in the field, with only a log line to go on.
      `IGNORE_FIX_QUALITY` is the same escape hatch `gps_signal` already ships for
      being equally undocumented (#86).

    This is the single definition of "is this a real position", shared with
    `capture_route.py` and `calibrate_heading.py`. A second copy is a second thing to
    get wrong, and two places disagreeing about one payload is what #76 and #77 both
    were.
    """
    if not ignore_fix_quality and _num(data, "fix_quality") == 0:
        return "GPS reports no fix (fix_quality 0)"
    lat, lon = _num(data, "latitude"), _num(data, "longitude")
    if (lat is not None and abs(lat) > 90.0) or (lon is not None and abs(lon) > 180.0):
        return f"position {lat}, {lon} is not on Earth"
    return None


def position_is_real(data, ignore_fix_quality=False):
    """True if `data`'s position is something a controller may steer on."""
    return no_fix_reason(data, ignore_fix_quality) is None


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

        # Is there a fix at all? Distinct from the freshness check below, which a
        # live link with no GPS lock sails straight through (#77).
        reason = no_fix_reason(
            data, getattr(c, "ignore_fix_quality", False))
        if reason:
            v.no_fix = True
            self._once("nofix", reason, v)

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
