"""A fresh clone must be able to run what the README tells it to run (issue #34).

`pip install -r requirements.txt` gave you `requests` and nothing else, so the very
next line of the README — running pytest — failed, and `--vision` died with a bare
ImportError instead of saying which requirements file it needed.
"""
import os
import re

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def reqs(name):
    with open(os.path.join(REPO, name)) as f:
        return [l.split("#")[0].strip() for l in f if l.split("#")[0].strip()]


def test_the_test_runner_is_declared_somewhere_installable():
    names = " ".join(reqs("requirements-dev.txt")).lower()
    assert "pytest" in names


def test_dev_requirements_pull_in_the_runtime_ones():
    assert any(l.startswith("-r requirements.txt") for l in reqs("requirements-dev.txt"))


def test_runtime_requirements_still_cover_the_follower():
    names = " ".join(reqs("requirements.txt")).lower()
    assert "requests" in names


def test_the_readme_setup_step_installs_the_test_runner():
    readme = open(os.path.join(REPO, "README.md")).read()
    setup = readme.split("## Setup", 1)[1].split("##", 1)[0]
    assert "requirements-dev.txt" in setup


def test_missing_vision_dependencies_produce_an_actionable_message():
    from waypoint_follower import vision_import_help
    msg = vision_import_help(ModuleNotFoundError("No module named 'torch'"))
    assert "vision/requirements.txt" in msg
    assert "torch" in msg


def test_every_declared_requirement_has_a_lower_bound():
    """An unpinned dependency is a future breakage nobody will be able to date."""
    for f in ("requirements.txt", "requirements-dev.txt", "vision/requirements.txt"):
        for line in reqs(f):
            if line.startswith("-r"):
                continue
            assert re.search(r"[><=~]=", line), f"{f}: {line} has no version bound"
