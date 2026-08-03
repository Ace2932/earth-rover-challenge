"""The SDK checkout must be reproducible off this laptop (issue #84).

Three fixes were needed to make the upstream SDK run at all, and all three lived
only in an untracked working copy. DEPLOYMENT.md told a VM to `git clone` the SDK
and nothing else, so a fresh clone gets none of them and fails in three different
ways — none of which points at the cause:

  1. `pip install -r requirements.txt` on Python 3.13+ has no wheels for the pinned
     aiohttp==3.9.3 / numpy==1.26.4 and dies compiling.
  2. `google-genai>=1.0.0` pulls a websockets version that conflicts with pyppeteer.
     It is only used by the optional TTS path.
  3. `browser_service.py` hardcodes `http://127.0.0.1:8000/sdk`, so setting
     SDK_PORT moves the API but not the page it drives. Everything 200s and no
     telemetry ever arrives — the worst failure of the three, because it looks like
     a bot problem.

These tests do not run the script (it needs network and a specific interpreter).
They pin the things that rot silently: that the script exists, fails loudly,
verifies rather than assumes, still covers all three divergences, and that the
deployment doc actually routes people through it.
"""
import os
import re
import stat

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(REPO, "deploy", "setup_sdk.sh")


def body():
    with open(SCRIPT) as f:
        return f.read()


def test_the_setup_script_exists():
    assert os.path.exists(SCRIPT)


def test_the_setup_script_is_executable():
    assert os.stat(SCRIPT).st_mode & stat.S_IXUSR, "chmod +x deploy/setup_sdk.sh"


def test_it_aborts_on_the_first_failure():
    """Half-applied setup is worse than none: it fails later, somewhere else."""
    assert re.search(r"set -[a-z]*e", body()), "needs `set -e` or equivalent"


def test_it_covers_the_python_version_divergence():
    assert "3.12" in body()


def test_it_covers_the_google_genai_conflict():
    assert "google-genai" in body()


def test_it_covers_the_hardcoded_sdk_port():
    """The one that looks like a bot problem rather than a setup problem."""
    b = body()
    assert "browser_service.py" in b and "SDK_PORT" in b


def test_it_verifies_each_fix_rather_than_assuming_it_applied():
    """A patch that silently no-ops on a changed upstream is how you find out on a
    sidewalk. Every fix has to be checked after it is applied."""
    b = body()
    assert b.count("verify") + b.count("VERIFY") >= 3, (
        "each of the three fixes must be verified after applying")


def test_it_does_not_hardcode_the_token():
    """The script runs on a VM someone else may read. It must take SDK_API_TOKEN from
    the environment, never carry one."""
    b = body()
    assert not re.search(r"SDK_API_TOKEN=[\"']?[A-Za-z0-9]{8,}", b)


# ---------------- the port fix must stay delegated and tested ----------------

def test_it_delegates_the_port_fix_to_the_tested_patcher():
    """The fix used to be a shell string replacement, checkable only by running it
    against the network — and it failed quietly twice that way. It lives in
    deploy/patch_browser_service.py now, with fixtures, so keep it there."""
    assert "patch_browser_service.py" in body(), (
        "the SDK_PORT fix is back in the shell script, where it cannot be tested")


def test_the_patcher_it_delegates_to_exists_and_is_executable():
    patcher = os.path.join(REPO, "deploy", "patch_browser_service.py")
    assert os.path.exists(patcher)
    assert os.stat(patcher).st_mode & stat.S_IXUSR


def test_the_script_does_not_reimplement_the_port_fix_inline():
    """Two copies of this patch is how a checkout ends up half-repaired. Comments may
    of course still explain what the patch is for — only executable lines count."""
    code = "\n".join(l for l in body().splitlines()
                     if l.strip() and not l.lstrip().startswith("#"))
    assert "127.0.0.1:8000/sdk" not in code, (
        "the script hardcodes the URL again instead of delegating")


# ---------------- the doc must route people through it ----------------

def test_the_deployment_doc_points_at_the_setup_script():
    with open(os.path.join(REPO, "DEPLOYMENT.md")) as f:
        doc = f.read()
    assert "setup_sdk.sh" in doc, (
        "DEPLOYMENT.md still tells a VM to clone the SDK with no fixes applied")


def test_the_deployment_doc_no_longer_offers_a_bare_clone_as_the_whole_step():
    """The exact trap: `git clone` immediately followed by a plain pip install."""
    with open(os.path.join(REPO, "DEPLOYMENT.md")) as f:
        doc = f.read()
    bare = re.search(r"git clone \S*earth-rovers-sdk\s*\n\s*(cd \S+ && )?pip3? install", doc)
    assert not bare, f"bare clone-then-install still in DEPLOYMENT.md: {bare.group(0)!r}"
