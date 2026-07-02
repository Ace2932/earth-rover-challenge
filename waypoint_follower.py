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
  python waypoint_follower.py --vision vision/sidewalk_policy.pt   # fuse sidewalk-keeping
"""
import argparse
import json
import math
import os
import time
from dataclasses import dataclass

from geo import haversine_m, bearing_deg, wrap180


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


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
    heading_min_move_m: float = 0.7    # move at least this to trust GPS course
    # magnetometer fallback: heading = SIGN*orientation*SCALE + OFFSET
    heading_scale: float = 360.0 / 255.0
    heading_offset: float = 0.0
    heading_sign: float = 1.0

    @classmethod
    def from_env(cls):
        c = cls()
        for f in c.__dataclass_fields__:
            env = os.getenv(f.upper())
            if env is not None:
                setattr(c, f, type(getattr(c, f))(env))
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


class HeadingEstimator:
    """Fuse heading: GPS course-over-ground when moving (drift-free, no calibration),
    magnetometer `orientation` when too slow for GPS course to be meaningful."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.anchor = None       # last GPS fix we computed course from

    def estimate(self, lat, lon, orientation):
        mag = (self.cfg.heading_sign * orientation * self.cfg.heading_scale
               + self.cfg.heading_offset) % 360.0
        if self.anchor is None:
            self.anchor = (lat, lon)
            return mag, "mag"
        moved = haversine_m(self.anchor[0], self.anchor[1], lat, lon)
        if moved >= self.cfg.heading_min_move_m:
            course = bearing_deg(self.anchor[0], self.anchor[1], lat, lon)
            self.anchor = (lat, lon)
            return course, "gps"
        return mag, "mag"


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
class LiveIO:
    def __init__(self, cfg):
        from rover_client import RoverClient
        self.c = RoverClient(base_url=os.getenv("SDK_BASE_URL", "http://localhost:8000"))
        self.h = HeadingEstimator(cfg)
        self.hsrc = "mag"

    def get_pose(self):
        d = self.c.get_data()
        lat, lon = float(d["latitude"]), float(d["longitude"])
        heading, self.hsrc = self.h.estimate(lat, lon, float(d.get("orientation", 0)))
        return lat, lon, heading

    def control(self, linear, angular):
        self.c.control(linear, angular)

    def front_frame(self):
        try:
            frame, _ = self.c.get_front_frame()
            return frame
        except Exception:
            return None

    def waypoints(self, route_file):
        if route_file:
            with open(route_file) as f:
                pts = json.load(f)
        else:
            self.c.start_mission()
            pts = self.c.checkpoints()["checkpoints_list"]
        return [(float(p["latitude"]), float(p["longitude"])) for p in pts]

    def reached(self):
        return self.c.checkpoint_reached()      # (ok, detail)


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
                (b[0] + 0.0000, b[1] + 0.0005)]

    def reached(self):
        return True, {}


def _fuse(gps_ang, vis_ang, gps_lin, vis_lin, alpha=0.5):
    return max(0.0, min(gps_lin, vis_lin)), clamp(alpha * gps_ang + (1 - alpha) * vis_ang, -1, 1)


def run(io, cfg, route_file=None, vision_fn=None, logger=None):
    wps = io.waypoints(route_file)
    print(f"[follower] {len(wps)} waypoints")
    is_mock = isinstance(io, MockIO)
    period = 1.0 / cfg.loop_hz
    i, step = 0, 0
    prev_ang = 0.0
    best_dist, t_best = math.inf, time.time()
    t0 = time.time()
    last_reach_try = 0.0
    try:
        while i < len(wps):
            now = time.time()
            if now - t0 > cfg.max_runtime_s:
                print("[follower] max runtime hit"); break
            lat, lon, heading = io.get_pose()
            tlat, tlon = wps[i]
            dist = haversine_m(lat, lon, tlat, tlon)
            brg = bearing_deg(lat, lon, tlat, tlon)

            # stuck detection (per waypoint)
            if dist < best_dist - 0.5:
                best_dist, t_best = dist, now
            elif now - t_best > cfg.stuck_s:
                print(f"[follower] STUCK on wp {i+1} ({dist:.1f}m, no progress {cfg.stuck_s}s)")
                break

            # server-authoritative checkpoint (rate-limited on the real bot to avoid
            # spamming the API; unthrottled in the mock, whose loop has no sleep)
            if dist < cfg.checkpoint_radius_m and (is_mock or now - last_reach_try > 0.8):
                last_reach_try = now
                ok, detail = io.reached()
                if ok:
                    io.control(0, 0)
                    print(f"[follower] reached wp {i+1}/{len(wps)} (dist {dist:.1f}m, step {step})")
                    i += 1
                    prev_ang, best_dist, t_best = 0.0, math.inf, time.time()
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
            io.control(linear, angular)

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
        io.control(0, 0)              # ALWAYS stop the rover, even on crash/ctrl-c
        if logger:
            logger.close()
    done = i >= len(wps)
    print(f"[follower] {'COMPLETE' if done else 'STOPPED'} — {i}/{len(wps)} waypoints, {step} steps")
    return done


def _load_vision(ckpt_path):
    import torch
    from io import BytesIO
    from PIL import Image
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "vision"))
    from policy import SidewalkPolicy
    ck = torch.load(ckpt_path, map_location="cpu", weights_only=False)
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="kinematic sim, no hardware")
    ap.add_argument("--route", help="JSON list of {latitude,longitude} (live mode)")
    ap.add_argument("--vision", help="path to a trained sidewalk_policy.pt to fuse")
    ap.add_argument("--log", help="write a per-step CSV to this path")
    args = ap.parse_args()

    cfg = Config.from_env()
    io = MockIO((37.8719, -122.2585, 0.0), cfg) if args.mock else LiveIO(cfg)
    vision_fn = _load_vision(args.vision) if args.vision else None
    logger = RunLogger(args.log) if args.log else None
    run(io, cfg, route_file=args.route, vision_fn=vision_fn, logger=logger)


if __name__ == "__main__":
    main()
