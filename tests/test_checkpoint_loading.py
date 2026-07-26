"""Loading a checkpoint must not execute it (issue #33).

`torch.load(..., weights_only=False)` runs arbitrary code from the file while
unpickling. That was defensible while checkpoints were only ever produced locally
by train.py; #26 adds a script that DOWNLOADS one, so the repo now has a
documented path from "curl a URL" to "execute what is in it".
"""
import os
import re

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOADERS = ["waypoint_follower.py", os.path.join("vision", "fuse.py")]


@pytest.mark.parametrize("path", LOADERS)
def test_no_loader_unpickles_arbitrary_objects(path):
    src = open(os.path.join(REPO, path)).read()
    for call in re.findall(r"torch\.load\([^)]*\)", src, re.S):
        assert "weights_only=False" not in call, f"{path}: {call.strip()}"


@pytest.mark.parametrize("path", LOADERS)
def test_every_loader_is_explicit_about_it(path):
    """torch changed this default in 2.6 — pin it rather than inherit it."""
    src = open(os.path.join(REPO, path)).read()
    for call in re.findall(r"torch\.load\([^)]*\)", src, re.S):
        assert "weights_only=True" in call, f"{path}: {call.strip()}"


def test_the_real_checkpoint_still_loads_with_the_safe_loader():
    torch = pytest.importorskip("torch")
    ckpt = os.path.join(REPO, "vision", "sidewalk_frodobots.pt")
    if not os.path.exists(ckpt):
        pytest.skip("checkpoint not present (see vision/fetch_model.sh)")
    ck = torch.load(ckpt, map_location="cpu", weights_only=True)
    assert "state_dict" in ck and ck["backbone"] == "resnet18"
