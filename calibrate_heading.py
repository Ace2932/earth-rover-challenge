"""Heading calibration — recover the orientation->degrees mapping for a real bot.

The SDK's `orientation` field isn't documented as degrees. This drives the rover
straight for a few seconds, takes the true heading from the GPS track, compares it
to the reported `orientation`, and prints the HEADING_SCALE/OFFSET/SIGN to put in
your env so waypoint_follower steers correctly. Do this ONCE per bot before racing.

Run (against the fake server or a real bot):
    SDK_BASE_URL=http://localhost:8777 python3 calibrate_heading.py
Safe: only drives straight forward at half throttle for CAL_SECS (default 4s).
Keep ~5 m of clear space ahead.
"""
import os
import time

from rover_client import RoverClient
from geo import bearing_deg, haversine_m, wrap180


def main():
    c = RoverClient(base_url=os.getenv("SDK_BASE_URL", "http://localhost:8000"))
    secs = float(os.getenv("CAL_SECS", "4"))
    hz = 5
    throttle = float(os.getenv("CAL_THROTTLE", "0.5"))
    print(f"driving straight {secs}s at linear={throttle} — keep the path clear")

    samples = []
    t0 = time.time()
    while time.time() - t0 < secs:
        d = c.get_data()
        samples.append((float(d["latitude"]), float(d["longitude"]), float(d["orientation"])))
        c.control(throttle, 0.0)
        time.sleep(1.0 / hz)
    c.control(0, 0)

    if len(samples) < 3:
        print("too few samples; increase CAL_SECS"); return
    la0, lo0, _ = samples[0]
    la1, lo1, _ = samples[-1]
    dist = haversine_m(la0, lo0, la1, lo1)
    if dist < 1.0:
        print(f"moved only {dist:.2f} m — raise CAL_SECS/CAL_THROTTLE; can't calibrate"); return

    B = bearing_deg(la0, lo0, la1, lo1)                 # true heading from GPS track
    Os = [s[2] for s in samples]
    O = sum(Os) / len(Os)
    SCALE = 360.0 / 255.0                               # assume orientation spans 0..255
    OFFSET = wrap180(B - O * SCALE)

    print(f"\n  GPS-track heading  B = {B:6.1f} deg   (moved {dist:.1f} m)")
    print(f"  raw orientation    O = {O:6.1f}       (range {min(Os):.0f}..{max(Os):.0f})")
    print("\n  recommended env (assumes 0..255 span, SIGN=+1):")
    print(f"    export HEADING_SCALE={SCALE:.7f}")
    print(f"    export HEADING_OFFSET={OFFSET:.1f}")
    print(f"    export HEADING_SIGN=1")
    print("\n  SIGN CHECK: start the follower; if it steers AWAY from the target,")
    print("  set HEADING_SIGN=-1 and HEADING_OFFSET to its negative, re-run.")


if __name__ == "__main__":
    main()
