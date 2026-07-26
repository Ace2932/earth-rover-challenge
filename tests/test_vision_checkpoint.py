"""The default --vision checkpoint has to be gettable (issue #13).

`--vision` with no argument loads `vision/sidewalk_frodobots.pt`, but `.gitignore`
contains `*.pt`, so the 43 MB file is untracked. On a fresh clone the documented
default printed "vision checkpoint not found" and exited — while the README
advertised the flag as working out of the box.
"""
import os

from waypoint_follower import DEFAULT_VISION, missing_checkpoint_help

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_the_help_tells_you_how_to_actually_get_the_model():
    msg = missing_checkpoint_help("vision/sidewalk_frodobots.pt")
    assert "fetch_model.sh" in msg
    assert "colab_frodobots.ipynb" in msg          # or train your own


def test_the_help_names_the_file_you_asked_for():
    assert "some/other.pt" in missing_checkpoint_help("some/other.pt")


def test_a_fetch_script_exists_and_is_executable():
    script = os.path.join(REPO, "vision", "fetch_model.sh")
    assert os.path.exists(script)
    assert os.access(script, os.X_OK), "fetch_model.sh is not executable"


def test_the_expected_checksum_is_recorded():
    """A model downloaded over the network is a model you have to verify."""
    sha = os.path.join(REPO, "vision", "sidewalk_frodobots.sha256")
    assert os.path.exists(sha)
    line = open(sha).read().split()
    assert len(line[0]) == 64, "not a sha256 digest"
    assert "sidewalk_frodobots.pt" in line[-1]


def test_the_model_card_records_provenance():
    card = open(os.path.join(REPO, "vision", "MODEL_CARD.md")).read()
    for field in ("resnet18", "96", "FrodoBots-2K", "sha256"):
        assert field in card, f"model card does not mention {field}"


def test_the_default_path_still_points_at_the_vision_directory():
    assert DEFAULT_VISION.endswith(os.path.join("vision", "sidewalk_frodobots.pt"))
