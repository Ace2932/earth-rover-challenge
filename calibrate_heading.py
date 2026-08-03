"""Heading calibration — recover the orientation->degrees mapping for a real bot.

The SDK's `orientation` field isn't documented as degrees. This drives the rover
straight for a few seconds, takes the true heading from the GPS track, compares it
to the reported `orientation`, and prints the HEADING_SCALE/OFFSET/SIGN to put in
your env so waypoint_follower steers correctly. Do this ONCE per bot before racing.

Run (against the fake server or a real bot):
    SDK_BASE_URL=http://localhost:8777 python3 calibrate_heading.py

SAFETY: this drives the rover forward at CAL_THROTTLE (default 0.5) for CAL_SECS
(default 4s). Keep ~5 m of clear space ahead and stay clear of the path. The stop
is in a `finally` and is retried, so a telemetry failure or a Ctrl-C still stops
the rover — but the rover latches its last command, so never kill -9 this script.
Use the follower's `--watchdog` if you want out-of-process protection too.
"""
import os
import time

from rover_client import RoverClient
from geo import bearing_deg, haversine_m, wrap180
from telemetry import ignore_fix_quality_from_env, position_is_real


def usable_samples(samples):
    """Drop samples whose position is not a real fix (#82).

    The all-sentinel run was already safe by luck — 1000/1000 throughout gives a
    haversine of 0, which trips the "moved only 0.00 m" guard below. The dangerous
    case is a lock that arrives or drops partway through the drive: the chord then
    runs between a real position and nowhere, the distance is enormous so the guard
    waves it through, and this prints a confident HEADING_OFFSET derived from a
    bearing to nowhere.

    A wrong calibration is worse than none. It is the seed the heading filter starts
    from, and the runbook has you exporting it by hand into every run that day.

    Uses the follower's own definition rather than a second copy of it — two places
    disagreeing about one payload is what #76 and #77 both were.
    """
    ignore_fq = ignore_fix_quality_from_env()
    return [s for s in samples
            if position_is_real({"latitude": s[0], "longitude": s[1]},
                                ignore_fix_quality=ignore_fq)]


def hard_stop(client, attempts=10, gap_s=0.05):
    """Stop the rover, and keep trying. A stop that fails silently is worse than no
    stop: this runs while someone is standing next to a rover under power."""
    for _ in range(attempts):
        try:
            client.control(0, 0)
            return True
        except Exception:
            time.sleep(gap_s)
    print("WARNING: could not confirm the rover stopped — stop it manually")
    return False


def collect(client, secs, hz=5.0, throttle=0.5, stop_attempts=10, stop_gap_s=0.05):
    """Drive straight and sample telemetry. Returns [(lat, lon, orientation), ...].

    The stop is in a `finally`, so a get_data() failure — which is a normal 4G
    event, and the reason RoverClient retries at all — or a Ctrl-C cannot leave the
    rover rolling forward at `throttle` with the process gone.
    """
    samples = []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < secs:
            d = client.get_data()
            samples.append((float(d["latitude"]), float(d["longitude"]),
                            float(d["orientation"])))
            client.control(throttle, 0.0)
            time.sleep(1.0 / hz)
    finally:
        hard_stop(client, attempts=stop_attempts, gap_s=stop_gap_s)
    return samples


def main():
    c = RoverClient(base_url=os.getenv("SDK_BASE_URL", "http://localhost:8000"))
    secs = float(os.getenv("CAL_SECS", "4"))
    throttle = float(os.getenv("CAL_THROTTLE", "0.5"))
    print(f"driving straight {secs}s at linear={throttle} — keep the path clear")

    raw = collect(c, secs=secs, throttle=throttle)
    samples = usable_samples(raw)
    dropped = len(raw) - len(samples)
    if dropped:
        print(f"dropped {dropped}/{len(raw)} samples with no GPS lock")

    if len(samples) < 3:
        print("too few samples with a real GPS fix; are you outdoors with a lock? "
              "(check `curl $SDK_BASE_URL/data` — latitude 1000 means no fix)")
        return
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
    print("\n  SIGN CHECK: start the follower; if the FIRST turn goes the wrong way,")
    print("  set HEADING_SIGN=-1 and HEADING_OFFSET to its negative, re-run.")
    print("  (Since the heading filter only SEEDS from the magnetometer and then")
    print("   corrects from GPS course, a bad seed costs one wrong turn, not the run.)")


if __name__ == "__main__":
    main()
