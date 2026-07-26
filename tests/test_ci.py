"""CI has to run the things that actually break (issue #35).

#29 is the cautionary tale: two PRs that each passed their own unit tests combined
into a silent regression, because nothing exercised the seam between them.
"""
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKFLOW = os.path.join(REPO, ".github", "workflows", "tests.yml")


def wf():
    return open(WORKFLOW).read()


def test_a_workflow_exists():
    assert os.path.exists(WORKFLOW)


def test_it_runs_on_pull_requests():
    assert "pull_request" in wf()


def test_it_runs_the_unit_suite():
    assert "pytest tests/" in wf()


def test_it_installs_the_test_runner():
    """Self-sufficient install: CI must not silently depend on #34 landing first."""
    assert "pytest" in wf()
    assert "requirements.txt" in wf()


def test_it_also_drives_the_real_client_end_to_end():
    """Unit tests mock the backend; the integration seam is where #29 lived."""
    text = wf()
    assert "fake_sdk_server.py" in text
    assert "COMPLETE" in text


def test_the_end_to_end_job_asserts_on_the_outcome():
    """A run that exits 0 without completing the mission is not a pass."""
    assert "grep -q" in wf()


def test_it_exercises_an_unreliable_link():
    assert "FAKE_FAIL_RATE" in wf()
