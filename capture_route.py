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
from telemetry import ignore_fix_quality_from_env, position_is_real

# Re-exported, not reimplemented: one definition of "is this a real position",
# shared with the follower's telemetry guard (#77).
usable_fix = position_is_real

# The shipped defaults, as constants rather than string literals buried in
# main(), so a test can assert the pair actually holds instead of asserting a
# relationship between one real default and one invented number (#87).
#
# Spacing sits above HEADING_MIN_MOVE_M (8 m) as well as above LOCAL_ARRIVE_M:
# waypoints closer than the heading filter's odometry baseline mean it never
# gets a GPS course correction between two of them, so the leg runs on dead
# reckoning.
DEFAULT_SPACING_M = 10.0
DEFAULT_HZ = 2.0


def _env_float(name, default):
    """`NAME=` with nothing after it is the ordinary shape of a .env line, and it
    reaches python as '' — for which os.getenv's two-arg default does NOT fire and
    float('') raises. Same trap as #84's SDK_PORT."""
    return float(os.getenv(name) or default)


def default_spacing_m():
    return _env_float("CAPTURE_SPACING_M", DEFAULT_SPACING_M)


def default_hz():
    return _env_float("CAPTURE_HZ", DEFAULT_HZ)


def default_arrive_m():
    """The FOLLOWER's arrival radius, read from its Config rather than retyped here.
    A second literal would warn about the wrong pair the moment the follower's
    default moved (#87)."""
    from waypoint_follower import Config
    return _env_float("LOCAL_ARRIVE_M", Config().local_arrive_m)


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

    # Same override the follower reads, so capture cannot drop fixes the
    # follower would accept — two places disagreeing about one payload is the
    # failure this shared rule exists to avoid (#86).
    ignore_fq = ignore_fix_quality_from_env()
    if ignore_fq:
        log('[capture] IGNORE_FIX_QUALITY set — keeping fixes that report '
            'fix_quality 0 (coordinate sanity still applies)')
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
            if not usable_fix(d, ignore_fix_quality=ignore_fq):
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
    capture(client, out, hz=default_hz(), spacing_m=default_spacing_m(),
            arrive_m=default_arrive_m())
    return 0


if __name__ == "__main__":
    sys.exit(main())
