"""Retired claims must not survive a merge (issue #46).

Auto-merge keeps both sides when two branches edit different regions of a file.
In code that has shown up as a doubled `io.get_pose()`, a doubled `io.control()`,
a clobbered `done` flag and two `torch.load` calls with the unsafe one first —
all caught by tests. Prose has no such safety net, and the README is what gets
read on call day.

So: a short list of claims that were true once, are false now, and would mislead
someone standing next to a rover.
"""
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RETIRED = [
    # #1: the pre-heading.py design, which measured ~88 deg median error
    ("README.md", "drift-free, no calibration",
     "describes the GPS-course-per-step estimator deleted in #18"),
    ("README.md", "uses GPS course-over-ground when moving",
     "same retired estimator"),
    # #40: the claim that could not be reproduced
    ("README.md", "verified surviving a 60%",
     "unverified; the measured figures are in the resilience bullet"),
]


def read(name):
    with open(os.path.join(REPO, name)) as f:
        return f.read()


def test_no_retired_claims_survive():
    live = [(f, claim, why) for f, claim, why in RETIRED if claim in read(f)]
    assert not live, "\n".join(f"{f}: {claim!r} — {why}" for f, claim, why in live)


def test_the_heading_section_is_not_duplicated():
    body = read("README.md")
    assert body.count("- **Heading") == 1, "two heading bullets: one of them is stale"


def test_the_safety_stop_claim_admits_what_it_cannot_do():
    """`try/finally` cannot survive kill -9 or a suspended laptop. Saying the rover
    'never runs away' is the kind of sentence that stops someone asking Frodobots
    whether the firmware has its own watchdog."""
    body = read("README.md")
    if "Safety-stop always" in body:
        assert "kill -9" in body or "SIGKILL" in body
