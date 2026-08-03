#!/usr/bin/env bash
# Set up a RUNNABLE earth-rovers-sdk checkout (issue #84).
#
# Upstream does not run as cloned. Three fixes are needed, all of them found the
# hard way during live bring-up on 2026-07-30, and all of them previously living
# only in an untracked working copy on one laptop:
#
#   1. Python version. requirements.txt pins aiohttp==3.9.3 and numpy==1.26.4,
#      which have no wheels for 3.13+ and die compiling. 3.12 is the newest that
#      works.
#   2. google-genai>=1.0.0 pulls a websockets version that conflicts with
#      pyppeteer. It is only used by the optional TTS path in tts_service.py.
#   3. browser_service.py hardcodes http://127.0.0.1:8000/sdk. Setting SDK_PORT
#      moves the API but NOT the page it drives, so every request 200s and no
#      telemetry ever arrives. This is the worst of the three because it looks
#      like a bot problem, not a setup problem.
#
# Idempotent: safe to re-run. Every fix is VERIFIED after it is applied, because a
# patch that silently no-ops against a changed upstream is something you would
# otherwise discover on a sidewalk.
#
# Usage:
#   deploy/setup_sdk.sh [target-dir]      # default: ./earth-rovers-sdk
#
# Then, with SDK_API_TOKEN and BOT_SLUG in <target>/.env:
#   cd <target> && .venv/bin/hypercorn main:app --bind 127.0.0.1:${SDK_PORT:-8000}
#
# The token is never written by this script. Put it in <target>/.env yourself;
# that file is gitignored by the SDK's own repo.

set -euo pipefail

REPO_URL="https://github.com/frodobots-org/earth-rovers-sdk"
TARGET="${1:-earth-rovers-sdk}"
PY="${SDK_PYTHON:-python3.12}"
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say() { printf '[setup-sdk] %s\n' "$*"; }
die() { printf '[setup-sdk] ERROR: %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- interpreter
command -v "$PY" >/dev/null 2>&1 || die \
  "$PY not found. requirements.txt pins aiohttp==3.9.3 / numpy==1.26.4, which have
  no wheels for Python 3.13+. Install 3.12 (brew install python@3.12 / apt install
  python3.12-venv) or set SDK_PYTHON to a 3.12 interpreter."

PYVER="$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
[ "$PYVER" = "3.12" ] || say "WARNING: $PY is $PYVER, not 3.12 — expect wheel failures"

# ---------------------------------------------------------------------- clone
if [ -d "$TARGET/.git" ]; then
  say "reusing existing checkout at $TARGET"
else
  say "cloning $REPO_URL -> $TARGET"
  git clone --depth 1 "$REPO_URL" "$TARGET"
fi
cd "$TARGET"

# ------------------------------------------------- fix 2: google-genai conflict
# Filter rather than edit requirements.txt, so `git pull` in this checkout stays
# clean and a re-run re-derives the filtered list from whatever upstream now says.
REQ_FILTERED=".requirements.no-genai.txt"
grep -v '^[[:space:]]*google-genai' requirements.txt > "$REQ_FILTERED"
# verify: the conflicting pin is gone, and we did not empty the file
grep -q '^[[:space:]]*google-genai' "$REQ_FILTERED" && die "google-genai survived the filter"
[ -s "$REQ_FILTERED" ] || die "filtered requirements are empty — check requirements.txt"
say "verify: google-genai excluded ($(wc -l < "$REQ_FILTERED" | tr -d ' ') requirements remain)"

# --------------------------------------------------------- fix 1: 3.12 venv
if [ ! -x .venv/bin/python ]; then
  say "creating .venv with $PY ($PYVER)"
  "$PY" -m venv .venv
fi
.venv/bin/pip install --quiet --upgrade pip
say "installing requirements (this pulls Chrome-driving pyppeteer; give it a minute)"
.venv/bin/pip install --quiet -r "$REQ_FILTERED"
# verify: the two pins that motivated the version choice actually imported
.venv/bin/python - <<'EOF' || exit 1
import sys
try:
    import aiohttp, numpy, pyppeteer          # noqa: F401
except Exception as e:                        # pragma: no cover - setup-time only
    sys.exit(f"[setup-sdk] ERROR: verify failed, imports broken after install: {e}")
print("[setup-sdk] verify: aiohttp, numpy and pyppeteer all import")
EOF

# ------------------------------------------------- fix 3: hardcoded SDK port
# Upstream drives its own page at a literal 127.0.0.1:8000, so SDK_PORT moves the
# API and leaves the page behind. Rewrite it to honour SDK_PORT, defaulting to the
# same 8000 so behaviour is unchanged when SDK_PORT is unset.
# Delegated to deploy/patch_browser_service.py so it can be tested against fixtures
# instead of only against the network. It handles a fresh upstream, repairs what an
# earlier version of this script left behind, is idempotent, and refuses rather than
# succeeding quietly when it cannot find its target. SDK_PORT is a `... or '8000'`
# fallback there, not os.getenv's two-arg default, because a bare `SDK_PORT=` line
# reaches python as '' and the two-arg form would not fire.
# It verifies itself by BEHAVIOUR — it evaluates the page URL the patched file will
# actually build, for SDK_PORT unset, set-but-empty and explicit — rather than
# grepping for a string. A file can mention SDK_PORT and still drop the port, which
# is the entire bug.
"$SETUP_DIR/patch_browser_service.py" browser_service.py \
  || die "could not make browser_service.py honour SDK_PORT (see above)"
grep -qE '^[[:space:]]*import os' browser_service.py \
  || die "browser_service.py uses os.getenv but never imports os"

# -------------------------------------------------------------------- summary
cat <<EOF

[setup-sdk] done — $TARGET is runnable.

Next:
  1. put your credentials in $TARGET/.env (gitignored by the SDK's own repo):
       SDK_API_TOKEN=...
       BOT_SLUG=...
       SDK_PORT=8001            # only if something else owns 8000
       # MISSION_SLUG=...       # LEAVE UNSET for a bot you own; see README
       CHROME_EXECUTABLE_PATH=...   # macOS: /Applications/Google Chrome.app/Contents/MacOS/Google Chrome
  2. run it:
       cd $TARGET && .venv/bin/hypercorn main:app --bind 127.0.0.1:\${SDK_PORT:-8000}
  3. point the follower at it:
       export SDK_BASE_URL=http://localhost:\${SDK_PORT:-8000}
EOF
