"""Urban-track GPS-waypoint follower for the Earth Rover Challenge.

Reads GPS + heading, steers toward the next checkpoint, cruises when aligned,
creeps on approach, and only advances when the SERVER confirms the checkpoint.
Built to survive a real 4G rover: always stops on exit, tolerates request
failures, fuses heading from GPS course (drift-free while moving) with the
magnetometer (when slow/stopped), detects being stuck, and logs every step.

Backends behind one interface (get_pose / control / waypoints / reached):
  --mock : kinematic sim, no hardware.
  live   : the real SDK server (rover_client.RoverClient), default.

Run:
  python waypoint_follower.py --mock
  python waypoint_follower.py                      # waypoints from /checkpoints-list
  python waypoint_follower.py --route route.json   # explicit [{latitude,longitude}]
  python waypoint_follower.py --vision                 # fuse the trained sidewalk policy
  python waypoint_follower.py --vision path/to/model.pt # or a specific checkpoint
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass

from blocked import BlockedGate
from envcfg import coerce
from geo import haversine_m, bearing_deg, wrap180
from heading import HeadingEstimator
from recovery import Recovery
from rover_client import server_distance
from telemetry import Guard


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MissionUnavailable(RuntimeError):
    """No usable route: the bot is not available, or the mission has no checkpoints."""


@dataclass
class Config:
    # The challenge accepts a checkpoint within 15 m and the server tells us its own
    # distance, so ask early and often rather than waiting for our own fix to agree.
    checkpoint_radius_m: float = 20.0  # start asking the server "reached?" inside this
    checkpoint_poll_s: float = 1.0     # how often to ask while inside it
    server_dist_max_age_s: float = 3.0 # after this, fall back to our own haversine
    cruise: float = 0.6                # forward throttle when aligned, 0..1
    kp_ang: float = 1.5                # steering gain (full turn near 45deg err)
    align_deg: float = 20.0            # within this err -> full cruise
    deadband_deg: float = 3.0          # ignore tiny heading errors (anti-jitter)
    approach_m: float = 6.0            # start slowing within this distance of a wp
    bearing_trust_m: float = 5.0       # below this the bearing is GPS noise, so damp
                                       # the turn rather than chase it (0 = disabled)
    min_creep: float = 0.25            # floor on the approach-slowdown factor
    max_dang: float = 0.35             # max angular change per step (slew limit)
    loop_hz: float = 5.0
    stuck_s: float = 20.0              # no progress this long -> stuck
    max_runtime_s: float = 3600.0

    # --- obstacle stop (needs a policy with a blocked head; see blocked.py) ---
    blocked_p: float = 0.8             # probability to call the path blocked; 1.0 = off
    blocked_frames: int = 2            # consecutive frames before braking
    blocked_hold_s: float = 1.5        # hold the stop this long (anti-chatter)
    # --- telemetry guards (see telemetry.py) ---
    battery_abort_pct: float = 15.0    # park rather than die somewhere inconvenient
    battery_warn_pct: float = 30.0
    gps_signal_good: float = 20.0      # units undocumented; good == poor disables this
    gps_signal_poor: float = 5.0
    min_speed_scale: float = 0.3       # slowest we will crawl on a bad fix
    no_motion_s: float = 4.0           # commanded to move this long with no wheel motion
    fix_max_age_s: float = 3.0         # /data timestamp frozen this long -> do not drive on it

    # --- vision fusion (only used with --vision) ---
    vision_alpha: float = 0.5          # weight on VISION steering (0 = pure GPS)
    vision_max_err_deg: float = 30.0   # above this the GPS controller is turning in
                                       # place; the policy has no useful opinion
    vision_min_linear: float = 0.25    # a timid BC policy must not pin us to a crawl
    frame_max_age_s: float = 0.7       # older than this -> do not steer on it
    # --- stuck recovery (see recovery.py) ---
    recovery_tries: int = 3            # back-up-and-turn attempts before a detour
    recovery_pause_s: float = 0.4
    recovery_reverse_s: float = 1.5
    recovery_reverse_throttle: float = 0.35
    recovery_yaw_s: float = 1.5
    recovery_yaw: float = 0.6
    recovery_offset_m: float = 8.0     # detour: approach from this far to the side
    detour_radius_m: float = 3.0       # close enough to the detour point
    detour_timeout_s: float = 45.0     # give up on the detour after this
    # --- command streaming (see commander.py) ---
    command_hz: float = 20.0           # rate the setpoint is re-sent to the bot
    setpoint_stale_s: float = 0.5      # setpoint older than this decays to a stop
    control_timeout_s: float = 1.0     # a stale command is worse than a dropped one
    data_timeout_s: float = 1.5

    # --- surviving a bad 4G link ---
    max_consecutive_errors: int = 20   # give up only after this many in a row
    stop_attempts: int = 10            # tries to get the rover stopped on the way out
    # --- heading estimation (see heading.py) ---
    heading_min_move_m: float = 8.0    # ODOMETRY baseline a course needs to beat the noise
    heading_max_turn_deg: float = 90.0 # NET turn past this and a chord says nothing
    heading_max_abs_turn_deg: float = 540.0  # loose sanity bound on total |yaw| (#54)
    heading_max_blind_s: float = 20.0  # no correction for this long -> take the next one anyway
    heading_gain: float = 0.25         # floor on the correction gain (see heading._gain)
    heading_slip_ratio: float = 0.75   # GPS chord below this * odometry = wheels slipping
    heading_jump_ratio: float = 1.5    # chord above this * odometry = the fix jumped (#61)
    heading_anchor_max_age_s: float = 30.0
    max_speed_mps: float = 1.5         # m/s at linear=1.0 (odometry model when telemetry
                                       # gives no `speed`)
    heading_min_linear: float = 0.05   # commanded throttle below this = not moving
    heading_min_speed: float = 0.05    # telemetry speed below this = not moving
    yaw_rate_dps: float = 90.0         # deg/s at angular=1.0 (dead-reckoning model)
    use_gyro: int = 0                  # 1 = trust /data gyros for yaw rate (verify units first)
    # magnetometer seed only, until the first GPS course lands:
    # heading = SIGN*orientation*SCALE + OFFSET
    heading_scale: float = 360.0 / 255.0
    heading_offset: float = 0.0
    heading_sign: float = 1.0

    # Ranges that a working mission requires. The runbook has you hand-editing .env
    # during live bring-up, and a stray minus sign or a zero meaning "off" is silently
    # accepted otherwise: CRUISE=-1 drives backwards at full throttle for the whole
    # mission, CHECKPOINT_RADIUS_M=0 never claims a checkpoint (#64).
    POSITIVE = ("loop_hz", "command_hz", "stuck_s", "max_runtime_s", "checkpoint_radius_m",
                "heading_min_move_m", "yaw_rate_dps", "max_speed_mps", "approach_m",
                "max_consecutive_errors", "stop_attempts", "recovery_tries")
    NON_NEGATIVE = ("checkpoint_poll_s", "server_dist_max_age_s", "deadband_deg",
                    "align_deg", "kp_ang", "max_dang", "setpoint_stale_s",
                    "control_timeout_s", "data_timeout_s", "fix_max_age_s",
                    "no_motion_s", "frame_max_age_s", "recovery_offset_m",
                    "detour_radius_m", "detour_timeout_s", "blocked_frames",
                    "blocked_hold_s", "heading_max_turn_deg", "heading_max_blind_s")
    UNIT_RANGE = ("cruise", "min_creep", "min_speed_scale", "vision_alpha",
                  "vision_min_linear", "blocked_p", "heading_gain", "recovery_yaw",
                  "recovery_reverse_throttle", "heading_slip_ratio")

    def validate(self):
        """Fail loudly at startup rather than driving under a config nobody checked."""
        for f in self.POSITIVE:
            if getattr(self, f, 1) <= 0:
                raise ValueError(f"{f.upper()} must be > 0, got {getattr(self, f)}")
        for f in self.NON_NEGATIVE:
            if getattr(self, f, 0) < 0:
                raise ValueError(f"{f.upper()} must be >= 0, got {getattr(self, f)}")
        for f in self.UNIT_RANGE:
            v = getattr(self, f, 0)
            if not 0.0 <= v <= 1.0:
                raise ValueError(f"{f.upper()} must be between 0 and 1, got {v}")
        return self

    @classmethod
    def from_env(cls):
        """Every field is overridable by its UPPERCASE name. See envcfg.coerce for
        why this does not just call `type(current)(value)`."""
        c = cls()
        for f in c.__dataclass_fields__:
            env = os.getenv(f.upper())
            if env is None:
                continue
            try:
                setattr(c, f, coerce(getattr(c, f), env))
            except ValueError as e:
                raise ValueError(f"{f.upper()}: {e}") from None
        return c.validate()


def steer(dist, bearing, heading, cfg):
    """Pure control law -> (linear, angular, err_deg).

    The bearing comes from two GPS points, so its reliability scales with how far
    apart they are. At sigma=1.5 m the bearing standard deviation is 2.5 deg at 30 m,
    14 deg at 5 m and 55 deg at 2 m — by then it carries no direction at all, and
    steering hard on it makes the rover spin on the spot chasing noise (#66). So
    below `bearing_trust_m` the turn is damped in proportion to how much of the
    bearing is still signal.
    """
    err = wrap180(bearing - heading)
    a = 0.0 if abs(err) < cfg.deadband_deg else cfg.kp_ang * err / 45.0
    angular = clamp(a, -1.0, 1.0)
    if cfg.bearing_trust_m > 0 and dist < cfg.bearing_trust_m:
        # Scale the COMMANDED turn, not the raw gain: at close range the gain is
        # saturated anyway, so damping before the clamp would barely change anything.
        angular *= clamp(dist / cfg.bearing_trust_m, 0.0, 1.0)
    if abs(err) > 90:
        linear = 0.0                                   # turn in place if pointing away
    elif abs(err) <= cfg.align_deg:
        linear = cfg.cruise
    else:
        linear = cfg.cruise * math.cos(math.radians(err))
    linear *= clamp(dist / cfg.approach_m, cfg.min_creep, 1.0)   # ease off on approach
    return linear, angular, err


class RunLogger:
    COLS = "t,wp,lat,lon,heading,hsrc,dist,sdist,bearing,err,linear,angular"

    def __init__(self, path):
        # Line-buffered: the log exists for post-mortem, and the failures worth
        # investigating are the ones that skip close() — kill -9, OOM, a suspended
        # machine. An unflushed log is empty in exactly those cases (#63).
        self.f = open(path, "w", buffering=1)
        self.f.write(self.COLS + "\n")

    def row(self, **k):
        self.f.write(",".join(f"{k.get(c, ''):.6f}" if isinstance(k.get(c), float)
                              else str(k.get(c, "")) for c in self.COLS.split(",")) + "\n")

    def close(self):
        self.f.close()


# ---------------- live backend ----------------
def _gyro_z(data, enabled):
    """Yaw rate from /data's `gyros`, or None. Off by default: the SDK does not
    document the axis order or the units, so verify on a real bot before trusting it."""
    if not enabled:
        return None
    try:
        return float(data["gyros"][-1][2])
    except (KeyError, IndexError, TypeError, ValueError):
        return None


class LiveIO:
    def __init__(self, cfg, heartbeat_path=None):
        from rover_client import RoverClient
        from commander import Commander
        base = os.getenv("SDK_BASE_URL", "http://localhost:8000")
        self.cfg = cfg
        self.c = RoverClient(base_url=base, timeout=cfg.data_timeout_s)
        # A separate client for commands: one attempt, short timeout. Retrying a
        # control message is pointless — the streamer sends a fresher one in 50 ms.
        self._cc = RoverClient(base_url=base, timeout=cfg.control_timeout_s, retries=1)
        self.cmd = Commander(lambda lin, ang: self._cc.control(lin, ang),
                             hz=cfg.command_hz, stale_s=cfg.setpoint_stale_s,
                             heartbeat_path=heartbeat_path)
        self.h = HeadingEstimator(cfg)
        self.hsrc = "mag"
        self.last_data = None
        self.last_cmd = (0.0, 0.0)      # what the estimator assumes is in force

    def get_pose(self):
        d = self.c.get_data()
        self.last_data = d          # the guards read the rest of the payload
        lat, lon = float(d["latitude"]), float(d["longitude"])
        speed = d.get("speed")
        heading, self.hsrc = self.h.update(
            lat, lon, float(d.get("orientation", 0)),
            cmd_linear=self.last_cmd[0], cmd_angular=self.last_cmd[1],
            gyro_z_dps=_gyro_z(d, self.cfg.use_gyro),
            speed=None if speed is None else float(speed))
        return lat, lon, heading

    def control(self, linear, angular):
        """Publish a setpoint. The Commander thread streams it; this never blocks."""
        self.last_cmd = (linear, angular)   # the heading estimator's motion gate (#29)
        self.cmd.set(linear, angular)

    def close(self):
        """Stop the rover, then check the telemetry agrees that it stopped."""
        self.cmd.close()
        first_ts = None
        frozen = False
        for _ in range(6):
            try:
                d = self.c.get_data()
            except Exception:
                time.sleep(0.3)
                continue
            # A frozen /data reports whatever speed it last saw, forever. That is not
            # evidence the rover is moving — it is the absence of evidence either way,
            # and saying "STILL MOVING" would be a claim we cannot support (#59).
            ts = d.get("timestamp")
            advanced = first_ts is not None and ts is not None and ts != first_ts
            if first_ts is None:
                first_ts = ts
            elif ts is not None and ts == first_ts:
                frozen = True
            speed = abs(float(d.get("speed", 0) or 0))
            if speed < 0.05 and not frozen:
                return True
            # Only claim the rover is still moving once we have seen the telemetry
            # advance — otherwise the speed we are reading may be a frozen echo.
            if advanced:
                print(f"[follower] STILL MOVING after stop (speed {speed:.2f}) — resending")
            try:
                self._cc.control(0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.3)
        if frozen:
            print("[follower] WARNING: telemetry is frozen — cannot confirm the rover "
                  "stopped. Stop commands were sent; verify the rover by eye.")
        else:
            print("[follower] WARNING: could not confirm the rover stopped")
        return False

    def front_frame(self):
        """Return (jpeg_bytes, timestamp) — the timestamp is how we spot a stalled
        stream serving the same frame over and over."""
        try:
            return self.c.get_front_frame()
        except Exception:
            return None, None

    def waypoints(self, route_file):
        """Return (waypoints, start_index). Raises MissionUnavailable rather than
        handing back an empty route, which the loop would report as success."""
        if route_file:
            with open(route_file) as f:
                pts = json.load(f)
            return [(float(p["latitude"]), float(p["longitude"])) for p in pts], 0

        ok, body = self.c.start_mission()
        if not ok and "unavailable" in str(body.get("detail", "")).lower():
            raise MissionUnavailable(
                f"/start-mission refused: {body.get('detail')}. The bot is not assigned "
                f"to this token, or another session holds it — check your allocation.")

        payload = self.c.checkpoints()
        pts = payload.get("checkpoints_list") or []
        if not pts:
            raise MissionUnavailable(
                "/checkpoints-list returned no checkpoints. Is MISSION_SLUG set and the "
                "mission started? (Refusing to 'complete' a zero-waypoint route.)")

        # `sequence` is the authority on order; payload order is not guaranteed.
        pts = sorted(pts, key=lambda p: int(p.get("sequence", 0)))
        wps = [(float(p["latitude"]), float(p["longitude"])) for p in pts]

        # Resume: the server counts how many checkpoints it has already accepted, so
        # that many are done. Only one SDK session may hold a bot, which makes a
        # crash-restart the only recovery path available — it has to resume correctly.
        start = clamp(int(payload.get("latest_scanned_checkpoint", 0) or 0), 0, len(wps))
        if start:
            print(f"[follower] resuming: server reports {start}/{len(wps)} already reached")
        return wps, start

    def reached(self):
        return self.c.checkpoint_reached()      # (ok, detail)

    def intervention(self, action):
        return self.c.intervention(action)


# ---------------- mock backend ----------------
class MockIO:
    MAX_SPEED = 1.5
    MAX_YAW = 90.0

    def __init__(self, start, cfg):
        self.lat, self.lon, self.heading = start
        self.dt = 1.0 / cfg.loop_hz
        self.hsrc = "sim"

    def get_pose(self):
        return self.lat, self.lon, self.heading % 360.0

    def control(self, linear, angular):
        self.heading = (self.heading + angular * self.MAX_YAW * self.dt) % 360.0
        speed = linear * self.MAX_SPEED
        dn = speed * self.dt * math.cos(math.radians(self.heading))
        de = speed * self.dt * math.sin(math.radians(self.heading))
        self.lat += dn / 111111.0
        self.lon += de / (111111.0 * math.cos(math.radians(self.lat)))

    def front_frame(self):
        return None, None

    def close(self):
        self.control(0, 0)

    def waypoints(self, route_file):
        # Honour --route here too: checking a route offline before driving it is
        # exactly what the mock is for, and silently substituting a canned square
        # tells you nothing about the route you asked about (#65).
        if route_file:
            with open(route_file) as f:
                pts = json.load(f)
            return [(float(p["latitude"]), float(p["longitude"])) for p in pts], 0
        b = (self.lat, self.lon)
        return [(b[0] + 0.0002, b[1] + 0.0001),
                (b[0] + 0.0003, b[1] + 0.0004),
                (b[0] + 0.0000, b[1] + 0.0005)], 0

    def reached(self):
        return True, {}


def frame_is_fresh(timestamp, now, cfg):
    """Is this camera frame recent enough to steer on?

    `/v2/front` serves "the latest emitted frame" from a ~500 ms WebRTC stream. If
    the stream stalls it keeps serving the same frame, and steering on a scene that
    no longer exists is worse than not steering at all. No timestamp -> we cannot
    tell, so allow it rather than disabling vision outright.
    """
    if timestamp is None:
        return True
    try:
        ts = float(timestamp)
    except (TypeError, ValueError):
        return True
    if ts > 1e11:               # milliseconds, not seconds
        ts /= 1000.0
    return (now - ts) <= cfg.frame_max_age_s


def fuse_gate(err_deg, confidence, cfg):
    """How much to trust vision right now, 0..1.

    Zero while the GPS controller is turning in place: at a large heading error the
    view is of whatever the rover happens to be sweeping past, which the policy has
    never been trained on. Scaled by the policy's own confidence so an
    out-of-distribution frame (night, rain, a different city) cannot outvote GPS.
    """
    if abs(err_deg) > cfg.vision_max_err_deg:
        return 0.0
    return clamp(cfg.vision_alpha * clamp(confidence, 0.0, 1.0), 0.0, 1.0)


def _fuse(gps_ang, vis_ang, gps_lin, vis_lin, alpha=0.5, min_linear=0.0):
    """Blend steering; take the more cautious speed, but never below `min_linear`
    unless the GPS controller itself wants a stop.

    The floor exists because `vis_lin` comes from behaviour cloning on hesitant
    human teleop, and MSE regression on a multi-modal target pulls toward the mean.
    A policy that outputs a small `linear` on most frames would otherwise pin the
    rover to a crawl for the whole mission — and score is difficulty x time.
    """
    angular = clamp((1 - alpha) * gps_ang + alpha * vis_ang, -1.0, 1.0)
    if gps_lin <= 0.0:
        return 0.0, angular                    # a deliberate stop stays a stop
    linear = max(min(min_linear, gps_lin), min(gps_lin, max(0.0, vis_lin)))
    return linear, angular


def server_distance_usable(detour, sdist, sdist_age_s, cfg):
    """May we navigate and measure progress on the server's distance right now?

    Only when it is fresh AND we are actually heading for the checkpoint it
    describes. While detouring we are deliberately driving away from that checkpoint
    to get around an obstacle, so its distance is the wrong yardstick: a
    correctly-executing detour would look like zero progress and re-trip stuck
    detection within seconds (#31).
    """
    if detour is not None:
        return False
    if sdist is None:
        return False
    return sdist_age_s < cfg.server_dist_max_age_s


def _intervene(io, action):
    """Record a human takeover. Best-effort: it is bookkeeping, not control, and a
    failure here must never stop us from stopping the rover."""
    fn = getattr(io, "intervention", None)
    if fn is None:
        return
    try:
        fn(action)
    except Exception as e:
        print(f"[follower] intervention {action} failed: {e}")
def safe_stop(io, cfg):
    """Stop the rover, retrying, swallowing everything.

    This runs in a `finally`. If it raises it does two bad things at once: it masks
    whatever actually went wrong, and it leaves the rover holding its last command
    — which on a bad link is exactly when the stop matters most.
    """
    for _ in range(getattr(cfg, "stop_attempts", 10)):
        try:
            io.control(0, 0)
            return True
        except Exception:
            time.sleep(0.05)
    print("[follower] WARNING: could not stop the rover — stop it manually")
    return False


def run(io, cfg, route_file=None, vision_fn=None, logger=None):
    wps, start = io.waypoints(route_file)
    if not wps:
        # Never report success for a route we never had. An empty list here means
        # the mission never started (see MissionUnavailable).
        print("[follower] no waypoints — refusing to run")
        safe_stop(io, cfg)
        return False
    print(f"[follower] {len(wps)} waypoints, starting at {start + 1}")
    is_mock = isinstance(io, MockIO)
    period = 1.0 / cfg.loop_hz
    i, step = start, 0
    prev_ang = 0.0
    best_dist, t_best = math.inf, time.monotonic()
    t0 = time.monotonic()
    last_reach_try = 0.0
    blocked_gate = BlockedGate(cfg)
    blocked_now = False
    guard = Guard(cfg)
    prev_lin = 0.0
    warned_no_motion = False
    aborted = False
    sdist, sdist_t = None, 0.0        # the server's own distance, and when it said so
    rec = Recovery(cfg)
    detour = None                 # (lat, lon, deadline) — a waypoint, NOT a checkpoint
    gave_up = False
    deadline = t0
    errors = 0                      # consecutive failed steps
    try:
        while i < len(wps):
            now = time.monotonic()
            if now - t0 > cfg.max_runtime_s:
                print("[follower] max runtime hit"); break
            try:
                lat, lon, heading = io.get_pose()
            except Exception as e:
                errors += 1
                if errors >= cfg.max_consecutive_errors:
                    print(f"[follower] link is down ({errors} failures in a row): {e}")
                    break
                # A skipped step is safe: the commander's setpoint decays to a stop.
                if errors == 1:
                    print(f"[follower] telemetry failed, retrying: {e}")
                if not is_mock:
                    time.sleep(period)
                continue

            # Telemetry guards: battery floor, GPS-quality speed scaling, and a
            # commanded-vs-actual motion check that notices a rover going nowhere
            # well before STUCK_S does.
            verdict = guard.check(getattr(io, "last_data", None), prev_lin, now)
            for w in verdict.warnings:
                print(f"[follower] WARNING: {w}")
            if verdict.abort:
                print(f"[follower] ABORT: {verdict.abort}")
                aborted = True
                break
            if verdict.stale_fix:
                # The position we would steer on is no longer real. Treat it exactly
                # like a failed telemetry read: skip the step, let the setpoint decay
                # to a stop, and give up deliberately if it never recovers. Driving on
                # a frozen fix ends in a recovery ladder planned against fiction (#59).
                errors += 1
                if errors >= cfg.max_consecutive_errors:
                    print("[follower] position fix has not updated — stopping")
                    break
                if not is_mock:
                    deadline = max(deadline + period, time.monotonic() - period)
                    time.sleep(max(0.0, deadline - time.monotonic()))
                continue

            # The guards above run BEFORE this branch on purpose: a manoeuvre must not
            # switch off the battery floor or the frozen-fix check. #59 exists to stop
            # the rover driving on a fix that is no longer real, and "a recovery ladder
            # planned against fiction" is the exact danger it named (#72).
            #
            # While recovering, the manoeuvre owns the rover: no steering, no
            # checkpoint claims, no stuck accounting.
            if rec.active:
                rlin, rang, still = rec.step(now)
                io.control(rlin, rang)
                if not still:
                    print(f"[follower] recovery attempt {rec.attempts} done — retrying")
                    best_dist, t_best, prev_ang = math.inf, now, 0.0
                step += 1
                if not is_mock:
                    deadline = max(deadline + period, time.monotonic() - period)
                    time.sleep(max(0.0, deadline - time.monotonic()))
                continue

            if detour and (now > detour[2] or
                           haversine_m(lat, lon, detour[0], detour[1]) < cfg.detour_radius_m):
                print("[follower] detour reached — approaching the checkpoint again")
                detour = None
                best_dist, t_best = math.inf, now
                sdist, sdist_t = None, 0.0         # target changed; ask again

            # The detour point IS the target while one is set — #52 had this line
            # overwritten by `tlat, tlon = wps[i]` in a silent auto-merge, which made
            # the whole detour a 45-second no-op driving at the obstacle.
            tlat, tlon = detour[:2] if detour else wps[i]
            dist = haversine_m(lat, lon, tlat, tlon)
            brg = bearing_deg(lat, lon, tlat, tlon)

            # Ask the server first, so its distance is available to everything below.
            # It decides whether the checkpoint counts, and its refusal carries the
            # distance it measured. Rate-limited so we do not spam the API (unthrottled
            # in the mock, whose loop has no sleep). Never asked while on a detour —
            # the detour point is not a checkpoint, and claiming it would be a lie.
            if not detour and dist < cfg.checkpoint_radius_m and (
                    is_mock or now - last_reach_try > cfg.checkpoint_poll_s):
                last_reach_try = now
                ok, detail = io.reached()
                if ok:
                    # Retried and swallowed: a dropped stop command must not cost us a
                    # checkpoint the server has already confirmed (#48). The next
                    # iteration commands again regardless.
                    safe_stop(io, cfg)
                    print(f"[follower] reached wp {i+1}/{len(wps)} (dist {dist:.1f}m, step {step})")
                    i += 1
                    prev_ang, best_dist, t_best = 0.0, math.inf, time.monotonic()
                    sdist, sdist_t = None, 0.0
                    rec = Recovery(cfg)          # fresh budget for the next leg
                    detour = None
                    continue
                d = server_distance(detail)
                if d is not None:
                    sdist, sdist_t = d, now

            if verdict.no_motion and not warned_no_motion:
                warned_no_motion = True
                print("[follower] commanded to move but no wheel motion — dropped "
                      "command, or the rover is held up on something")

            # Navigate on the server's distance while it is fresh — it is the number
            # that decides whether the checkpoint counts. Bearing still comes from our
            # own fix; the server only tells us how far, never which way.
            #
            # NOT while detouring (#31): that distance is to the CHECKPOINT, but we are
            # deliberately driving away from it to get around an obstacle. Measuring
            # progress against it would make a correctly-executing detour look like no
            # progress at all and re-trip stuck detection within seconds.
            fresh = server_distance_usable(detour, sdist, now - sdist_t, cfg)
            nav_dist = sdist if fresh else dist

            # stuck detection (per waypoint), measured the same way
            if nav_dist < best_dist - 0.5:
                best_dist, t_best = nav_dist, now
            elif now - t_best > cfg.stuck_s:
                print(f"[follower] STUCK on wp {i+1} ({nav_dist:.1f}m, "
                      f"no progress {cfg.stuck_s}s)")
                if not rec.exhausted:
                    rec.begin(now)
                    print(f"[follower] recovery attempt {rec.attempts}/{cfg.recovery_tries}: "
                          f"backing up and turning")
                    continue
                if cfg.recovery_offset_m > 0 and not detour:
                    # Approach from a different angle: aim at a point beside the
                    # checkpoint. It is not a checkpoint, so it is never claimed.
                    side = bearing_deg(lat, lon, *wps[i]) + 90.0
                    dlat = cfg.recovery_offset_m * math.cos(math.radians(side)) / 111111.0
                    dlon = (cfg.recovery_offset_m * math.sin(math.radians(side))
                            / (111111.0 * math.cos(math.radians(lat))))
                    detour = (wps[i][0] + dlat, wps[i][1] + dlon, now + cfg.detour_timeout_s)
                    print(f"[follower] recovery exhausted — detouring "
                          f"{cfg.recovery_offset_m:.0f}m to the side of wp {i+1}")
                    best_dist, t_best = math.inf, now
                    sdist, sdist_t = None, 0.0     # stale the moment the target changes
                    continue
                print(f"[follower] cannot free the rover on wp {i+1} — recording an "
                      f"intervention and stopping")
                _intervene(io, "start")
                gave_up = True
                break

            linear, angular, err = steer(nav_dist, brg, heading, cfg)
            linear *= verdict.speed_scale
            if vision_fn is not None:
                vf, vts = io.front_frame()
                if vf is None:
                    vsrc = "no-frame"
                elif not frame_is_fresh(vts, now, cfg):
                    vsrc = "stale"          # stalled stream: steer on GPS alone
                    if not warned_stale_frames:
                        warned_stale_frames = True
                        print("[follower] camera frames are stale — GPS-only steering")
                else:
                    # Four values, because confidence and P(blocked) are different
                    # questions: "how much should this steer us" versus "may we go
                    # forward at all". Merging them would let a confident stop read as
                    # a confident steer.
                    vlin, vang, vconf, vblocked = vision_fn(vf)
                    alpha = fuse_gate(err, vconf, cfg) if vlin is not None else 0.0
                    vsrc = f"a{alpha:.2f}"
                    if alpha > 0:
                        linear, angular = _fuse(angular, vang, linear, vlin,
                                                alpha=alpha,
                                                min_linear=cfg.vision_min_linear)
                    # After fusion: a stop overrides whatever the blend produced, and
                    # is not subject to VISION_MIN_LINEAR's floor.
                    if blocked_gate.update(vblocked, now):
                        # Steering still applies — stopped and pointed the right way
                        # beats stopped and lost.
                        linear = 0.0
                        vsrc = "blocked"
                        if not blocked_now:
                            blocked_now = True
                            print(f"[follower] BLOCKED (p={vblocked:.2f}) — holding")
                    elif blocked_now:
                        blocked_now = False
                        print("[follower] path clear — resuming")
            angular = clamp(angular, prev_ang - cfg.max_dang, prev_ang + cfg.max_dang)  # slew
            prev_ang, prev_lin = angular, linear
            try:
                io.control(linear, angular)
                errors = 0                      # a full clean step
            except Exception as e:
                errors += 1
                if errors >= cfg.max_consecutive_errors:
                    print(f"[follower] link is down ({errors} failures in a row): {e}")
                    break

            if logger:
                logger.row(t=now - t0, wp=i + 1, lat=lat, lon=lon, heading=heading,
                           hsrc=io.hsrc, dist=dist, sdist=("" if sdist is None else sdist),
                           bearing=brg, err=err, linear=linear, angular=angular)
            if step % max(1, int(cfg.loop_hz)) == 0:
                sd = f" srv={sdist:5.1f}m" if fresh else ""
                print(f"  wp{i+1} dist={dist:6.1f}m{sd} brg={brg:5.1f} hdg={heading:5.1f}"
                      f"[{io.hsrc}] err={err:+6.1f} lin={linear:+.2f} ang={angular:+.2f}")
            step += 1
            if not is_mock:
                # Hold the loop rate: sleeping a fixed period AFTER the work makes the
                # real rate `period + work`, which drifts as the link slows down.
                deadline = max(deadline + period, time.monotonic() - period)
                time.sleep(max(0.0, deadline - time.monotonic()))
    finally:
        # ALWAYS stop the rover — crash, Ctrl-C, or a clean finish. A backend with a
        # Commander does the hardened version (retried stop plus a telemetry read-back
        # confirming speed actually fell to zero); safe_stop covers the rest. Neither
        # may raise here: this is a finally, and raising would mask the real failure
        # AND leave the rover moving.
        # Always command a stop ourselves first, then let the backend tear down. Do
        # NOT delegate the stop to close() alone: run() cannot know that a given
        # backend's close() stops the rover, and a safety path must not rest on that
        # assumption.
        try:
            safe_stop(io, cfg)
        except Exception as e:
            print(f"[follower] stop failed: {e}")
        try:
            closer = getattr(io, "close", None)
            if closer:
                closer()
        except Exception as e:
            print(f"[follower] backend close failed: {e}")

        if logger:
            logger.close()
    # Neither a telemetry abort (flat battery) nor giving up after the recovery
    # ladder is a completed mission, and an empty route was never a mission at all.
    done = bool(wps) and i >= len(wps) and not aborted and not gave_up
    print(f"[follower] {'COMPLETE' if done else 'STOPPED'} — {i}/{len(wps)} waypoints, {step} steps")
    return done


def vision_import_help(exc):
    """--vision needs the ML stack, which is deliberately not in requirements.txt so
    GPS-only navigation stays a `pip install requests` away. Say which file to
    install rather than surfacing a bare ImportError."""
    return (f"--vision needs dependencies that are not installed: {exc}\n"
            f"  pip install -r vision/requirements.txt\n"
            f"    (torch, torchvision, pillow, opencv — a few hundred MB)\n"
            f"GPS-only navigation needs none of them: drop --vision.")


def _load_vision(ckpt_path):
    import torch
    from io import BytesIO
    from PIL import Image
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "vision"))
    from policy import SidewalkPolicy
    ck = torch.load(ckpt_path, map_location="cpu", weights_only=True)
    model = SidewalkPolicy(backbone=ck["backbone"],
                           blocked_head=bool(ck.get("blocked_head")))
    model.load_state_dict(ck["state_dict"])
    size = ck["img"]

    def infer(frame_bytes):
        """Return (linear, angular, confidence, p_blocked).

        Confidence and P(blocked) are deliberately separate: "how much should this
        steer us" and "may we go forward at all" are different questions, and folding
        them together would let a confident stop read as a confident steer.
        """
        try:
            im = Image.open(BytesIO(frame_bytes)).convert("RGB").resize((size, size))
        except Exception:
            return None, None, 0.0, None
        t = torch.tensor(list(im.getdata()), dtype=torch.float32).view(size, size, 3)
        t = (t / 255.0).permute(2, 0, 1)
        lin, ang, p_blocked = model.act(t)
        # Confidence and P(blocked) are separate answers. There is no uncertainty head
        # yet, so confidence is 1.0 and the err-gate plus VISION_ALPHA do the gating
        # (#11); p_blocked is None unless the checkpoint carries the stop head (#7).
        return lin, ang, 1.0, p_blocked
    return infer


DEFAULT_VISION = os.path.join(os.path.dirname(__file__), "vision", "sidewalk_frodobots.pt")


def missing_checkpoint_help(path):
    """The checkpoint is 43 MB and .gitignore excludes *.pt, so a fresh clone does
    not have it. Say how to get one instead of just reporting its absence."""
    return (f"vision checkpoint not found: {path}\n"
            f"  download it:  bash vision/fetch_model.sh\n"
            f"  or train one: vision/colab_frodobots.ipynb (full dataset), or see\n"
            f"                vision/README.md for a local run on the sample data\n"
            f"  what it is:   vision/MODEL_CARD.md — read the limitations first\n"
            f"(GPS-only navigation works without it: drop --vision.)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="kinematic sim, no hardware")
    ap.add_argument("--route", help="JSON list of {latitude,longitude} (live mode)")
    ap.add_argument("--vision", nargs="?", const=DEFAULT_VISION, default=None,
                    help=f"fuse a trained sidewalk policy; bare --vision loads "
                         f"{os.path.relpath(DEFAULT_VISION)}, or pass a checkpoint path")
    ap.add_argument("--log", help="write a per-step CSV to this path")
    ap.add_argument("--heartbeat", default=os.getenv("HEARTBEAT_PATH"),
                    help="file the command streamer touches on every delivered command")
    ap.add_argument("--watchdog", action="store_true",
                    help="also run watchdog.py in a separate process; it stops the rover "
                         "if this one dies without cleaning up (kill -9, OOM, sleep)")
    args = ap.parse_args()

    if args.vision and not os.path.exists(args.vision):
        print(missing_checkpoint_help(args.vision))
        return

    try:
        cfg = Config.from_env()
    except ValueError as e:
        # A traceback is the wrong thing to hand someone standing next to a rover.
        print(f"[follower] bad configuration: {e}\n"
              f"  check your .env / exported vars against .env.example")
        raise SystemExit(2)
    # Load the policy BEFORE spawning the watchdog: a missing torch used to be able
    # to return from main() with a watchdog subprocess already running, orphaning a
    # process that holds the bot's only SDK session.
    try:
        vision_fn = _load_vision(args.vision) if args.vision else None
    except ImportError as e:
        print(vision_import_help(e))
        return

    heartbeat = args.heartbeat or (os.path.join(tempfile.gettempdir(), "erc_follower.hb")
                                   if args.watchdog else None)
    watchdog = None
    if args.watchdog and not args.mock:
        watchdog = subprocess.Popen(
            [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                          "watchdog.py"),
             "--heartbeat", heartbeat, "--base-url",
             os.getenv("SDK_BASE_URL", "http://localhost:8000")])
        print(f"[follower] watchdog pid {watchdog.pid} on {heartbeat}")
    elif args.watchdog:
        print("[follower] --watchdog ignored in --mock (no rover to run away)")

    io = (MockIO((37.8719, -122.2585, 0.0), cfg) if args.mock
          else LiveIO(cfg, heartbeat_path=heartbeat))
    logger = RunLogger(args.log) if args.log else None
    try:
        run(io, cfg, route_file=args.route, vision_fn=vision_fn, logger=logger)
    except MissionUnavailable as e:
        print(f"[follower] cannot start: {e}")
        raise SystemExit(2)
    finally:
        # The watchdog exits by itself when Commander.close() removes the heartbeat;
        # only force it if it has not noticed. This must run on the MissionUnavailable
        # path too, or a refused mission leaves an orphan process holding the bot.
        if watchdog:
            try:
                watchdog.wait(timeout=5)
            except subprocess.TimeoutExpired:
                watchdog.terminate()


if __name__ == "__main__":
    main()
