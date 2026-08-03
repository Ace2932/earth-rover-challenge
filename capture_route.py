"""Turn a hand-driven lap into a route file for `waypoint_follower.py --route`.

A bot you own has no mission, and without one the SDK's mission API is unusable:
`/checkpoints-list` returns `{}` and `/checkpoint-reached` 500s. So there are no
server checkpoints to follow, and `--route` needs a file nothing in this repo
produced. This makes one: drive the bot by hand once, keep the track, hand it
back.

    # 1. teleop the bot however you normally do (SDK web UI, phone, controller)
    # 2. in another terminal, record the track:
    SDK_BASE_URL=http://localhost:8001 python3 capture_route.py park_lap.json
    # 3. Ctrl-C when the lap is done, then drive it autonomously:
    python3 waypoint_follower.py --route park_lap.json --watchdog --log run1.csv

SAFETY: this tool only ever READS. It never sends a control command, because a
human is holding the controls while it runs and a setpoint from a second process
is the one failure that could take the rover away from them. There is deliberately
no code path here that can drive.

Fixes the follower would refuse to steer on are dropped, using the follower's own
rule (`telemetry.position_is_real`) rather than a second copy of it — a route
quietly seeded with the 1000/1000 no-fix sentinel is worse than no route at all.
"""
import json
import os
import sys
import time

from geo import haversine_m
from rover_client import RoverClient
from telemetry import position_is_real

# Re-exported, not reimplemented: one definition of "is this a real position",
# shared with the follower's telemetry guard (#77).
usable_fix = position_is_real


def decimate(points, spacing_m):
    """Thin a dense track to waypoints at least `spacing_m` apart.

    The last point is always kept, however close it fell to the previous one: the
    route has to end where the driver stopped, or the follower's final leg aims at
    somewhere the rover was never actually driven.
    """
    if spacing_m <= 0:
        raise ValueError(f"spacing must be > 0, got {spacing_m}")
    if not points:
        return []
    kept = [points[0]]
    for p in points[1:]:
        if haversine_m(kept[-1][0], kept[-1][1], p[0], p[1]) >= spacing_m:
            kept.append(p)
    if kept[-1] != points[-1]:
        kept.append(points[-1])
    return kept


def spacing_warning(spacing_m, arrive_m):
    """Complain if waypoints are packed closer than the follower's arrival radius.

    Both numbers are individually reasonable; together they mean every waypoint is
    "reached" the instant the one before it is, so the rover skips the lot and cuts
    the corner it was recorded to follow. The #74 shape, in a new pair.
    """
    if spacing_m <= arrive_m:
        return (f"waypoint spacing {spacing_m:.0f} m is not greater than the "
                f"follower's arrival radius {arrive_m:.0f} m (LOCAL_ARRIVE_M): every "
                f"waypoint would be 'reached' the moment the one before it was. "
                f"Raise CAPTURE_SPACING_M above it.")
    return None


def to_route(points):
    """The exact JSON shape `waypoint_follower.waypoints()` reads back."""
    return [{"latitude": la, "longitude": lo} for la, lo in points]


def capture(client, out_path, hz, spacing_m, arrive_m, log=print):
    """Poll /data until interrupted, then write the decimated route. Read-only."""
    warn = spacing_warning(spacing_m, arrive_m)
    if warn:
        log(f"[capture] WARNING: {warn}")

    track, skipped, last_report = [], 0, 0.0
    log(f"[capture] recording at {hz:g} Hz — drive the bot now, Ctrl-C when done")
    try:
        while True:
            try:
                d = client.get_data()
            except Exception as e:                  # 4G drops are normal, not fatal
                log(f"[capture] telemetry failed, retrying: {e}")
                time.sleep(1.0 / hz)
                continue
            if not usable_fix(d):
                skipped += 1
            else:
                track.append((float(d["latitude"]), float(d["longitude"])))
            now = time.monotonic()
            if now - last_report > 2.0:
                last_report = now
                log(f"[capture] {len(track)} fixes, {skipped} skipped for no lock, "
                    f"{_track_length_m(track):.0f} m driven")
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        log("")

    if not track:
        log("[capture] no usable fixes — nothing written. Was the bot outdoors "
            "with a GPS lock?")
        return None

    route = decimate(track, spacing_m)
    with open(out_path, "w") as f:
        json.dump(to_route(route), f, indent=2)
    log(f"[capture] {len(track)} fixes ({_track_length_m(track):.0f} m) -> "
        f"{len(route)} waypoints -> {out_path}")
    if skipped:
        log(f"[capture] {skipped} samples dropped for having no GPS lock")
    log(f"[capture] drive it: python3 waypoint_follower.py --route {out_path} "
        f"--watchdog --log run1.csv")
    return route


def _track_length_m(track):
    return sum(haversine_m(a[0], a[1], b[0], b[1])
               for a, b in zip(track, track[1:]))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 2
    out = argv[0]
    client = RoverClient(base_url=os.getenv("SDK_BASE_URL", "http://localhost:8000"))
    # Default spacing sits above HEADING_MIN_MOVE_M (8 m): waypoints closer than the
    # heading filter's odometry baseline mean it never gets a GPS course correction
    # between them, so the whole leg runs on dead reckoning.
    capture(client, out,
            hz=float(os.getenv("CAPTURE_HZ", "2")),
            spacing_m=float(os.getenv("CAPTURE_SPACING_M", "10")),
            arrive_m=float(os.getenv("LOCAL_ARRIVE_M", "5")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
