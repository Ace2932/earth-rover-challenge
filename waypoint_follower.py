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
import time
from dataclasses import dataclass

from envcfg import coerce
from geo import haversine_m, bearing_deg, wrap180
from heading import HeadingEstimator
from recovery import Recovery


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MissionUnavailable(RuntimeError):
    """No usable route: the bot is not available, or the mission has no checkpoints."""


@dataclass
class Config:
    checkpoint_radius_m: float = 5.0   # gate to start asking the server "reached?"
    cruise: float = 0.6                # forward throttle when aligned, 0..1
    kp_ang: float = 1.5                # steering gain (full turn near 45deg err)
    align_deg: float = 20.0            # within this err -> full cruise
    deadband_deg: float = 3.0          # ignore tiny heading errors (anti-jitter)
    approach_m: float = 6.0            # start slowing within this distance of a wp
    min_creep: float = 0.25            # floor on the approach-slowdown factor
    max_dang: float = 0.35             # max angular change per step (slew limit)
    loop_hz: float = 5.0
    stuck_s: float = 20.0              # no progress this long -> stuck
    max_runtime_s: float = 3600.0

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
    # --- surviving a bad 4G link ---
    max_consecutive_errors: int = 20   # give up only after this many in a row
    stop_attempts: int = 10            # tries to get the rover stopped on the way out
    # --- heading estimation (see heading.py) ---
    heading_min_move_m: float = 8.0    # ODOMETRY baseline a course needs to beat the noise
    heading_max_turn_deg: float = 90.0 # past this a chord tells you nothing about heading
    heading_max_blind_s: float = 20.0  # no correction for this long -> take the next one anyway
    heading_gain: float = 0.25         # floor on the correction gain (see heading._gain)
    heading_slip_ratio: float = 0.75   # GPS chord below this * odometry = wheels slipping
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
        return c


def steer(dist, bearing, heading, cfg):
    """Pure control law -> (linear, angular, err_deg)."""
    err = wrap180(bearing - heading)
    a = 0.0 if abs(err) < cfg.deadband_deg else cfg.kp_ang * err / 45.0
    angular = clamp(a, -1.0, 1.0)
    if abs(err) > 90:
        linear = 0.0                                   # turn in place if pointing away
    elif abs(err) <= cfg.align_deg:
        linear = cfg.cruise
    else:
        linear = cfg.cruise * math.cos(math.radians(err))
    linear *= clamp(dist / cfg.approach_m, cfg.min_creep, 1.0)   # ease off on approach
    return linear, angular, err


class RunLogger:
    COLS = "t,wp,lat,lon,heading,hsrc,dist,bearing,err,linear,angular"

    def __init__(self, path):
        self.f = open(path, "w")
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
    def __init__(self, cfg):
        from rover_client import RoverClient
        self.cfg = cfg
        self.c = RoverClient(base_url=os.getenv("SDK_BASE_URL", "http://localhost:8000"))
        self.h = HeadingEstimator(cfg)
        self.hsrc = "mag"
        self.last_cmd = (0.0, 0.0)      # what the estimator assumes is in force

    def get_pose(self):
        d = self.c.get_data()
        lat, lon = float(d["latitude"]), float(d["longitude"])
        speed = d.get("speed")
        heading, self.hsrc = self.h.update(
            lat, lon, float(d.get("orientation", 0)),
            cmd_linear=self.last_cmd[0], cmd_angular=self.last_cmd[1],
            gyro_z_dps=_gyro_z(d, self.cfg.use_gyro),
            speed=None if speed is None else float(speed))
        return lat, lon, heading

    def control(self, linear, angular):
        self.last_cmd = (linear, angular)
        self.c.control(linear, angular)

    def front_frame(self):
        try:
            frame, _ = self.c.get_front_frame()
            return frame
        except Exception:
            return None

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
        return None

    def waypoints(self, route_file):
        b = (self.lat, self.lon)
        return [(b[0] + 0.0002, b[1] + 0.0001),
                (b[0] + 0.0003, b[1] + 0.0004),
                (b[0] + 0.0000, b[1] + 0.0005)], 0

    def reached(self):
        return True, {}


def _fuse(gps_ang, vis_ang, gps_lin, vis_lin, alpha=0.5):
    return max(0.0, min(gps_lin, vis_lin)), clamp(alpha * gps_ang + (1 - alpha) * vis_ang, -1, 1)


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
    best_dist, t_best = math.inf, time.time()
    t0 = time.time()
    last_reach_try = 0.0
    rec = Recovery(cfg)
    detour = None                 # (lat, lon, deadline) — a waypoint, NOT a checkpoint
    gave_up = False
    errors = 0                      # consecutive failed steps
    try:
        while i < len(wps):
            now = time.time()
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
                    time.sleep(period)
                continue

            if detour and (now > detour[2] or
                           haversine_m(lat, lon, detour[0], detour[1]) < cfg.detour_radius_m):
                print("[follower] detour reached — approaching the checkpoint again")
                detour = None
                best_dist, t_best = math.inf, now

            tlat, tlon = detour[:2] if detour else wps[i]
            tlat, tlon = wps[i]
            dist = haversine_m(lat, lon, tlat, tlon)
            brg = bearing_deg(lat, lon, tlat, tlon)

            # stuck detection (per waypoint)
            if dist < best_dist - 0.5:
                best_dist, t_best = dist, now
            elif now - t_best > cfg.stuck_s:
                print(f"[follower] STUCK on wp {i+1} ({dist:.1f}m, no progress {cfg.stuck_s}s)")
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
                    continue
                print(f"[follower] cannot free the rover on wp {i+1} — recording an "
                      f"intervention and stopping")
                _intervene(io, "start")
                gave_up = True
                break

            # server-authoritative checkpoint (rate-limited on the real bot to avoid
            # spamming the API; unthrottled in the mock, whose loop has no sleep).
            # Never asked while on a detour: the detour point is not a checkpoint.
            if not detour and dist < cfg.checkpoint_radius_m and (
                    is_mock or now - last_reach_try > 0.8):
                last_reach_try = now
                ok, detail = io.reached()
                if ok:
                    # Retried and swallowed: a dropped stop command must not cost us a
                    # checkpoint the server has already confirmed (#48). The next
                    # iteration commands again regardless.
                    safe_stop(io, cfg)
                    print(f"[follower] reached wp {i+1}/{len(wps)} (dist {dist:.1f}m, step {step})")
                    i += 1
                    prev_ang, best_dist, t_best = 0.0, math.inf, time.time()
                    rec = Recovery(cfg)          # fresh budget for the next leg
                    detour = None
                    continue

            linear, angular, err = steer(dist, brg, heading, cfg)
            if vision_fn is not None:
                vf = io.front_frame()
                if vf is not None:
                    vlin, vang = vision_fn(vf)
                    if vlin is not None:
                        linear, angular = _fuse(angular, vang, linear, vlin)
            angular = clamp(angular, prev_ang - cfg.max_dang, prev_ang + cfg.max_dang)  # slew
            prev_ang = angular
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
                           hsrc=io.hsrc, dist=dist, bearing=brg, err=err,
                           linear=linear, angular=angular)
            if step % max(1, int(cfg.loop_hz)) == 0:
                print(f"  wp{i+1} dist={dist:6.1f}m brg={brg:5.1f} hdg={heading:5.1f}[{io.hsrc}] "
                      f"err={err:+6.1f} lin={linear:+.2f} ang={angular:+.2f}")
            step += 1
            if not is_mock:
                time.sleep(period)
    finally:
        safe_stop(io, cfg)            # ALWAYS stop the rover, even on crash/ctrl-c
        if logger:
            logger.close()
    done = i >= len(wps) and not gave_up
    done = bool(wps) and i >= len(wps)
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
    model = SidewalkPolicy(backbone=ck["backbone"])
    model.load_state_dict(ck["state_dict"])
    size = ck["img"]

    def infer(frame_bytes):
        try:
            im = Image.open(BytesIO(frame_bytes)).convert("RGB").resize((size, size))
        except Exception:
            return None, None
        t = torch.tensor(list(im.getdata()), dtype=torch.float32).view(size, size, 3)
        t = (t / 255.0).permute(2, 0, 1)
        return model.act(t)
    return infer


DEFAULT_VISION = os.path.join(os.path.dirname(__file__), "vision", "sidewalk_frodobots.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="kinematic sim, no hardware")
    ap.add_argument("--route", help="JSON list of {latitude,longitude} (live mode)")
    ap.add_argument("--vision", nargs="?", const=DEFAULT_VISION, default=None,
                    help=f"fuse a trained sidewalk policy; bare --vision loads "
                         f"{os.path.relpath(DEFAULT_VISION)}, or pass a checkpoint path")
    ap.add_argument("--log", help="write a per-step CSV to this path")
    args = ap.parse_args()

    if args.vision and not os.path.exists(args.vision):
        print(f"vision checkpoint not found: {args.vision}\n"
              f"train one (Colab: vision/colab_frodobots.ipynb) or place "
              f"sidewalk_frodobots.pt in vision/.")
        return

    cfg = Config.from_env()
    io = MockIO((37.8719, -122.2585, 0.0), cfg) if args.mock else LiveIO(cfg)
    try:
        vision_fn = _load_vision(args.vision) if args.vision else None
    except ImportError as e:
        print(vision_import_help(e))
        return
    logger = RunLogger(args.log) if args.log else None
    try:
        run(io, cfg, route_file=args.route, vision_fn=vision_fn, logger=logger)
    except MissionUnavailable as e:
        print(f"[follower] cannot start: {e}")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
