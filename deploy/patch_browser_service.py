#!/usr/bin/env python3
"""Make the SDK's `browser_service.py` honour SDK_PORT (issue #84).

Upstream drives its own page at a literal `http://127.0.0.1:8000/sdk`, so setting
SDK_PORT moves the API but not the page. Every request still 200s and no telemetry
ever arrives — the worst of the SDK's three setup problems, because it looks like a
bot problem rather than a setup problem.

This is a separate file rather than a `sed` inside `deploy/setup_sdk.sh` because a
string replacement can only be checked by running it against the network, and it
failed quietly in two ways that only showed up when it was:

  - against an upstream whose line has changed at all it matches nothing, does
    nothing, and the shell's `verify` step passes anyway;
  - a checkout patched by an EARLIER version keeps that version's bug forever,
    because the literal being searched for is already gone.

`tests/test_patch_browser_service.py` runs this against fixtures for both.

Exit codes: 0 patched or already correct, 1 could not (with a reason on stderr).

    deploy/patch_browser_service.py path/to/browser_service.py
"""
import ast
import pathlib
import re
import sys

# What we want. `or` and not os.getenv's two-arg default, deliberately: a bare
# `SDK_PORT=` line — the ordinary shape of a .env placeholder — reaches python as
# '', and the two-arg default only fires when the name is ABSENT. The URL would
# become `http://127.0.0.1:/sdk`: the same silent no-telemetry failure this patch
# exists to prevent, reintroduced by the patch itself.
GOOD = "f\"http://127.0.0.1:{os.getenv('SDK_PORT') or '8000'}/sdk\""

# Upstream's literal, and the earlier patched form that needs repairing. Both are
# matched loosely on whitespace and quote style so an upstream reformat does not
# silently defeat us.
PATTERNS = (
    re.compile(r"""["']http://127\.0\.0\.1:8000/sdk["']"""),
    re.compile(r"""f["']http://127\.0\.0\.1:\{\s*os\.getenv\(\s*["']SDK_PORT["']\s*,"""
               r"""\s*["']8000["']\s*\)\s*\}/sdk["']"""),
)


def page_url_expression(src):
    """The compiled f-string that builds the SDK page URL.

    NOT "the first JoinedStr in the file": upstream's browser_service.py contains
    other f-strings, and the first one interpolates module-level names that do not
    exist at verify time. Taking it evaluates an unrelated expression and dies with
    a NameError — which the first version of this verify did, against a test fixture
    that happened to contain only one f-string.
    """
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.JoinedStr) and any(
                isinstance(v, ast.Constant) and "127.0.0.1" in str(v.value)
                for v in node.values):
            return compile(ast.Expression(node), "<page-url>", "eval")
    raise ValueError("no page-URL f-string found in the file")


def verify(src):
    """Evaluate the page URL under each shape SDK_PORT arrives in.

    Checked by BEHAVIOUR rather than by grepping for a string, because a file can
    mention SDK_PORT and still drop the port — which is the whole bug.
    """
    import os as _os
    code = page_url_expression(src)
    saved = _os.environ.pop("SDK_PORT", None)
    try:
        for value, want in ((None, ":8000/sdk"), ("", ":8000/sdk"), ("8001", ":8001/sdk")):
            _os.environ.pop("SDK_PORT", None)
            if value is not None:
                _os.environ["SDK_PORT"] = value
            url = eval(code, {"os": _os})
            if want not in url:
                raise ValueError(f"SDK_PORT={value!r} builds {url!r}, wanted {want!r}")
    finally:
        _os.environ.pop("SDK_PORT", None)
        if saved is not None:
            _os.environ["SDK_PORT"] = saved


def already_correct(src):
    """True if the file honours SDK_PORT *and* falls back on a set-but-empty one."""
    return GOOD in src


def patch(src):
    """Return the patched source, or None if no known form was found."""
    if already_correct(src):
        return src
    for pat in PATTERNS:
        new, n = pat.subn(lambda _: GOOD, src, count=1)
        if n:
            return new
    return None


def main(argv):
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 1
    path = pathlib.Path(argv[1])
    if not path.is_file():
        print(f"ERROR: {path} does not exist — is this an earth-rovers-sdk checkout?",
              file=sys.stderr)
        return 1

    src = path.read_text()
    if already_correct(src):
        try:
            verify(src)
        except ValueError as e:
            print(f"ERROR: {path} looked patched but does not behave: {e}",
                  file=sys.stderr)
            return 1
        print(f"[patch] {path.name} already honours SDK_PORT — nothing to do")
        return 0

    out = patch(src)
    if out is None:
        print(f"ERROR: could not find the page URL in {path}. Upstream has changed "
              f"the line this patch targets, so SDK_PORT would move the API but not "
              f"the page it drives: every request 200s and no telemetry arrives. "
              f"Patch it by hand to {GOOD} and open an issue.", file=sys.stderr)
        return 1

    try:
        ast.parse(out)
    except SyntaxError as e:
        print(f"ERROR: patching {path} would not parse ({e}) — refusing to write",
              file=sys.stderr)
        return 1

    if not re.search(r"^\s*import os\b", out, re.M):
        print(f"ERROR: {path} uses os.getenv but never imports os", file=sys.stderr)
        return 1

    try:
        verify(out)
    except ValueError as e:
        print(f"ERROR: the patched {path} would not behave correctly ({e}) — "
              f"refusing to write", file=sys.stderr)
        return 1

    path.write_text(out)
    print(f"[patch] {path.name} now honours SDK_PORT "
          f"(verified for unset, empty and explicit)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
