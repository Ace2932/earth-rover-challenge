""".env.example must not contradict itself (issue #50).

Every feature branch appended its own block, git kept both sides of every
non-overlapping region, and the file ended up defining HEADING_MIN_MOVE_M three
times — twice with 0.7, the pre-#18 value that measured ~88 deg of heading error.

Which one wins depends on who reads the file. Someone copying a line out of it by
hand during live bring-up — exactly what the runbook asks for — gets whichever
they see first.
"""
import os
import re

from waypoint_follower import Config

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSIGNMENT = re.compile(r"^([A-Z][A-Z0-9_]*)=")


def assignments():
    out = []
    with open(os.path.join(REPO, ".env.example")) as f:
        for n, line in enumerate(f, 1):
            m = ASSIGNMENT.match(line)
            if m:
                out.append((m.group(1), n))
    return out


def test_no_key_is_defined_twice():
    seen = {}
    dupes = []
    for key, line in assignments():
        if key in seen:
            dupes.append(f"{key}: lines {seen[key]} and {line}")
        else:
            seen[key] = line
    assert not dupes, "duplicate keys:\n  " + "\n  ".join(dupes)


def test_every_follower_key_maps_to_a_real_config_field():
    """A typo'd key is silently ignored by Config.from_env, so the example file is
    the only place it can be caught."""
    fields = {f.upper() for f in Config().__dataclass_fields__}
    # keys another tool owns, not the follower's Config: the SDK server, the fake
    # server (FAKE_*, skipped below) and capture_route.py
    external = {"SDK_API_TOKEN", "BOT_SLUG", "CHROME_EXECUTABLE_PATH", "MISSION_SLUG",
                "SDK_BASE_URL", "HEARTBEAT_PATH",
                "CAPTURE_HZ", "CAPTURE_SPACING_M"}
    unknown = [k for k, _ in assignments()
               if k not in fields and k not in external and not k.startswith("FAKE_")]
    assert not unknown, f"not Config fields and not external: {unknown}"


def test_the_retired_heading_baseline_is_gone():
    """0.7 m is the chord length from #1. It must not appear as a suggested value."""
    body = open(os.path.join(REPO, ".env.example")).read()
    assert "HEADING_MIN_MOVE_M=0.7" not in body
