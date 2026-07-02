"""Urban-track GPS-waypoint follower for the Earth Rover Challenge.

A proportional bearing controller: read GPS + heading, compute bearing to the
next checkpoint, steer to null the heading error, cruise forward when aligned,
turn in place when far off. Reaches a checkpoint within CHECKPOINT_RADIUS_M,
advances, repeats.

Two IO backends behind one interface (get_pose / control / mission calls):
  --mock  : a kinematic sim (no hardware) so you can validate the controller now.
  live    : the real SDK server (rover_client.RoverClient), default.

Run:
  python waypoint_follower.py --mock            # sim a short route, watch it converge
  python waypoint_follower.py                    # live, GPS waypoints from /checkpoints-list
  python waypoint_follower.py --route route.json # live, explicit [{latitude,longitude}] list

Tune via env or flags. On a real bot, CALIBRATE the heading mapping first
(HEADING_SCALE/OFFSET/SIGN) — the SDK `orientation` units are not documented as
degrees; drive straight and compare GPS-track bearing to the reported value.
"""
import argparse
import json
import math
import os
import time

from geo import haversine_m, bearing_deg, wrap180


# ---- tunables (env-overridable) ----
CHECKPOINT_RADIUS_M = float(os.getenv("CHECKPOINT_RADIUS_M", "5.0"))
CRUISE = float(os.getenv("CRUISE", "0.6"))          # forward throttle when aligned, 0..1
KP_ANG = float(os.getenv("KP_ANG", "1.5"))          # steering gain
ALIGN_DEG = float(os.getenv("ALIGN_DEG", "20"))     # within this err -> allow full cruise
LOOP_HZ = float(os.getenv("LOOP_HZ", "5"))
# real-bot heading calibration: heading_deg = SIGN*orientation*SCALE + OFFSET
HEADING_SCALE = float(os.getenv("HEADING_SCALE", str(360.0 / 255.0)))
HEADING_OFFSET = float(os.getenv("HEADING_OFFSET", "0"))
HEADING_SIGN = float(os.getenv("HEADING_SIGN", "1"))


def steer(dist, bearing, heading):
    """Return (linear, angular, err_deg) for one control step."""
    err = wrap180(bearing - heading)
    angular = max(-1.0, min(1.0, KP_ANG * err / 45.0))   # saturate near 45deg err
    if abs(err) > 90:
        linear = 0.0                                     # turn in place if pointing away
    elif abs(err) <= ALIGN_DEG:
        linear = CRUISE
    else:
        linear = CRUISE * math.cos(math.radians(err))    # ease off while turning
    return linear, angular, err


# ---------------- live backend ----------------
class LiveIO:
    def __init__(self):
        from rover_client import RoverClient
        self.c = RoverClient(base_url=os.getenv("SDK_BASE_URL", "http://localhost:8000"))

    def get_pose(self):
        d = self.c.get_data()
        heading = HEADING_SIGN * d["orientation"] * HEADING_SCALE + HEADING_OFFSET
        return float(d["latitude"]), float(d["longitude"]), heading % 360.0

    def control(self, linear, angular):
        self.c.control(linear, angular)

    def waypoints(self, route_file):
        if route_file:
            with open(route_file) as f:
                pts = json.load(f)
        else:
            self.c.start_mission()
            pts = self.c.checkpoints()["checkpoints_list"]
        return [(float(p["latitude"]), float(p["longitude"])) for p in pts]

    def reached(self):
        try:
            self.c.checkpoint_reached()
        except Exception:
            pass  # proximity is server-enforced; local radius already gated us


# ---------------- mock backend ----------------
class MockIO:
    """Kinematic sim. Heading 0=N,90=E. Integrates control at LOOP_HZ."""
    MAX_SPEED = 1.5        # m/s at linear=1
    MAX_YAW = 90.0         # deg/s at angular=1

    def __init__(self, start):
        self.lat, self.lon, self.heading = start
        self.dt = 1.0 / LOOP_HZ

    def get_pose(self):
        return self.lat, self.lon, self.heading % 360.0

    def control(self, linear, angular):
        self.heading = (self.heading + angular * self.MAX_YAW * self.dt) % 360.0
        speed = linear * self.MAX_SPEED
        dn = speed * self.dt * math.cos(math.radians(self.heading))
        de = speed * self.dt * math.sin(math.radians(self.heading))
        self.lat += dn / 111111.0
        self.lon += de / (111111.0 * math.cos(math.radians(self.lat)))

    def waypoints(self, route_file):
        # a short synthetic route ~30-40m NE of start
        b = (self.lat, self.lon)
        return [(b[0] + 0.0002, b[1] + 0.0001),
                (b[0] + 0.0003, b[1] + 0.0004),
                (b[0] + 0.0000, b[1] + 0.0005)]

    def reached(self):
        pass


def run(io, route_file=None, max_steps=100000):
    wps = io.waypoints(route_file)
    print(f"[follower] {len(wps)} waypoints")
    i = 0
    step = 0
    period = 1.0 / LOOP_HZ
    while i < len(wps) and step < max_steps:
        lat, lon, heading = io.get_pose()
        tlat, tlon = wps[i]
        dist = haversine_m(lat, lon, tlat, tlon)
        if dist < CHECKPOINT_RADIUS_M:
            io.control(0, 0)
            io.reached()
            print(f"[follower] reached wp {i+1}/{len(wps)} (dist {dist:.1f}m, step {step})")
            i += 1
            continue
        brg = bearing_deg(lat, lon, tlat, tlon)
        linear, angular, err = steer(dist, brg, heading)
        io.control(linear, angular)
        if step % max(1, int(LOOP_HZ)) == 0:
            print(f"  wp{i+1} dist={dist:6.1f}m brg={brg:5.1f} hdg={heading:5.1f} "
                  f"err={err:+6.1f} lin={linear:+.2f} ang={angular:+.2f}")
        step += 1
        if isinstance(io, MockIO):
            continue  # sim runs as fast as possible
        time.sleep(period)
    io.control(0, 0)
    done = i >= len(wps)
    print(f"[follower] {'COMPLETE' if done else 'STOPPED'} — {i}/{len(wps)} waypoints, {step} steps")
    return done


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="run kinematic sim, no hardware")
    ap.add_argument("--route", help="JSON list of {latitude,longitude} (live mode)")
    args = ap.parse_args()
    if args.mock:
        io = MockIO(start=(37.8719, -122.2585, 0.0))  # near UC Berkeley
    else:
        io = LiveIO()
    run(io, route_file=args.route)


if __name__ == "__main__":
    main()
