"""The SDK port patch, as a thing that can be tested (issue #84).

The patch that makes `browser_service.py` honour SDK_PORT used to be a shell string
replacement, which meant the only way to know it worked was to run it against the
network and read the output. Two failure modes came out of that immediately:

  - it targets a literal, so against an upstream that has changed the line at all it
    silently does nothing and the `verify` step passes anyway;
  - a checkout patched by an EARLIER version of the script keeps that earlier
    version's bug forever, because the literal it searched for is already gone.

So the patch lives here instead, and these tests run it against fixtures.

The bug the earlier version had is worth stating plainly, because it is the same
silent failure the patch exists to prevent: `os.getenv('SDK_PORT', '8000')` does not
fall back for a bare `SDK_PORT=` line — python-dotenv makes that `''`, the two-arg
default only fires when the name is ABSENT, and the URL becomes
`http://127.0.0.1:/sdk`. The page never loads, every request still 200s, and no
telemetry ever arrives.
"""
import os
import subprocess
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATCHER = os.path.join(REPO, "deploy", "patch_browser_service.py")

# Shaped like the real file, which matters: upstream's browser_service.py contains
# OTHER f-strings, and the first one in the file is not the page URL. A verifier that
# takes "the first JoinedStr" evaluates an unrelated expression referencing
# module-level names and dies with a NameError — which is what the shell script's
# first verify did, against a fixture here that only ever had one f-string.
UPSTREAM = '''import os

FORMAT = os.getenv("IMAGE_FORMAT", "png")


class BrowserService:
    async def screenshot(self, name):
        return f"screenshots/{name}.{FORMAT}"

    async def start(self):
        await self.page.goto(
            "http://127.0.0.1:8000/sdk", {"waitUntil": "networkidle2"}
        )
'''

# what the first version of the script produced: correct for a set port, wrong for
# a set-but-empty one
OLD_PATCH = UPSTREAM.replace(
    '"http://127.0.0.1:8000/sdk"',
    "f\"http://127.0.0.1:{os.getenv('SDK_PORT', '8000')}/sdk\"")

ALREADY_GOOD = UPSTREAM.replace(
    '"http://127.0.0.1:8000/sdk"',
    "f\"http://127.0.0.1:{os.getenv('SDK_PORT') or '8000'}/sdk\"")


def run_patch(tmp_path, source):
    f = tmp_path / "browser_service.py"
    f.write_text(source)
    p = subprocess.run([sys.executable, PATCHER, str(f)],
                       capture_output=True, text=True)
    return p, f


sys.path.insert(0, os.path.join(REPO, "deploy"))
import patch_browser_service as pbs  # noqa: E402


def url_from(path, sdk_port):
    """Evaluate the patched file's page URL under `sdk_port`.

    Uses the patcher's own expression finder rather than a second copy: "the first
    f-string in the file" is wrong on the real upstream, and a test helper that got
    it right independently would hide the day the patcher got it wrong.
    """
    saved = os.environ.pop("SDK_PORT", None)
    try:
        if sdk_port is not None:
            os.environ["SDK_PORT"] = sdk_port
        code = pbs.page_url_expression(open(path).read())
        return eval(code, {"os": os})
    finally:
        os.environ.pop("SDK_PORT", None)
        if saved is not None:
            os.environ["SDK_PORT"] = saved


# ---------------- patching upstream ----------------

def test_it_patches_a_fresh_upstream_checkout(tmp_path):
    p, f = run_patch(tmp_path, UPSTREAM)
    assert p.returncode == 0, p.stderr
    assert "SDK_PORT" in f.read_text()


def test_the_patched_file_honours_an_explicit_port(tmp_path):
    _, f = run_patch(tmp_path, UPSTREAM)
    assert url_from(f, "8001") == "http://127.0.0.1:8001/sdk"


def test_the_patched_file_defaults_when_sdk_port_is_unset(tmp_path):
    _, f = run_patch(tmp_path, UPSTREAM)
    assert url_from(f, None) == "http://127.0.0.1:8000/sdk"


def test_the_patched_file_defaults_when_sdk_port_is_set_but_empty(tmp_path):
    """`SDK_PORT=` is the ordinary shape of a .env placeholder line."""
    _, f = run_patch(tmp_path, UPSTREAM)
    assert url_from(f, "") == "http://127.0.0.1:8000/sdk"


# ---------------- repairing what an older run left behind ----------------

def test_it_repairs_a_checkout_patched_by_the_earlier_version(tmp_path):
    """The literal is already gone, so a search-and-replace on it no-ops and the
    old bug survives every re-run. This is the case that made the patch worth
    extracting from the shell script at all."""
    p, f = run_patch(tmp_path, OLD_PATCH)
    assert p.returncode == 0, p.stderr
    assert url_from(f, "") == "http://127.0.0.1:8000/sdk"


def test_it_is_idempotent_on_an_already_correct_file(tmp_path):
    p, f = run_patch(tmp_path, ALREADY_GOOD)
    assert p.returncode == 0, p.stderr
    assert f.read_text() == ALREADY_GOOD, "rewrote a file that was already correct"


def test_running_it_twice_changes_nothing_the_second_time(tmp_path):
    _, f = run_patch(tmp_path, UPSTREAM)
    once = f.read_text()
    subprocess.run([sys.executable, PATCHER, str(f)], check=True,
                   capture_output=True, text=True)
    assert f.read_text() == once


# ---------------- refusing to succeed quietly ----------------

def test_it_fails_loudly_on_an_upstream_it_does_not_recognise(tmp_path):
    """A patch that cannot find its target must say so. Passing silently here is how
    you get a rover that 200s on every request and never reports a position."""
    p, _ = run_patch(tmp_path, "import os\nURL = compute_it_some_other_way()\n")
    assert p.returncode != 0
    assert "SDK_PORT" in (p.stderr + p.stdout)


def test_it_refuses_a_file_that_would_not_parse_afterwards(tmp_path):
    p, _ = run_patch(tmp_path, 'def broken(:\n    "http://127.0.0.1:8000/sdk"\n')
    assert p.returncode != 0


def test_the_url_expression_is_found_among_other_f_strings(tmp_path):
    """Upstream has other f-strings, and the first one in the file is not the page
    URL — it interpolates a module-level name that does not exist at verify time.
    Taking 'the first JoinedStr' raises NameError against the real file."""
    assert "f\"screenshots/" in UPSTREAM, "fixture must contain a decoy f-string first"
    _, f = run_patch(tmp_path, UPSTREAM)
    assert "127.0.0.1" in str(eval(pbs.page_url_expression(f.read_text()), {"os": os}))


def test_it_reports_a_missing_file_rather_than_creating_one(tmp_path):
    p = subprocess.run([sys.executable, PATCHER, str(tmp_path / "nope.py")],
                       capture_output=True, text=True)
    assert p.returncode != 0
    assert not (tmp_path / "nope.py").exists()
