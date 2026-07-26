#!/usr/bin/env bash
# Fetch the trained sidewalk-keeping checkpoint that `--vision` loads by default.
#
# The file is 43 MB and .gitignore excludes *.pt, so it does not live in the repo.
# It comes from a GitHub Release asset, and is verified against the sha256 recorded
# next to this script.
#
#   bash vision/fetch_model.sh                    # default release asset
#   VISION_MODEL_URL=https://... bash vision/fetch_model.sh
#
# If you would rather not download anything, train your own — see
# vision/colab_frodobots.ipynb, or vision/README.md for the local quick run.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$DIR/sidewalk_frodobots.pt"
SHA_FILE="$DIR/sidewalk_frodobots.sha256"
REPO="${VISION_MODEL_REPO:-Ace2932/earth-rover-challenge}"
TAG="${VISION_MODEL_TAG:-vision-v1}"
URL="${VISION_MODEL_URL:-https://github.com/$REPO/releases/download/$TAG/sidewalk_frodobots.pt}"

expected="$(awk '{print $1}' "$SHA_FILE")"

verify() {
  if command -v sha256sum >/dev/null 2>&1; then
    actual="$(sha256sum "$OUT" | awk '{print $1}')"
  else
    actual="$(shasum -a 256 "$OUT" | awk '{print $1}')"
  fi
  [ "$actual" = "$expected" ] || {
    echo "CHECKSUM MISMATCH" >&2
    echo "  expected $expected" >&2
    echo "  actual   $actual" >&2
    echo "Refusing to leave an unverified model in place; removing it." >&2
    rm -f "$OUT"
    exit 1
  }
}

if [ -f "$OUT" ]; then
  echo "already present: $OUT"
  verify && echo "checksum OK"
  exit 0
fi

echo "downloading $URL"
if ! curl -fL --progress-bar -o "$OUT" "$URL"; then
  rm -f "$OUT"
  cat >&2 <<EOF

Download failed. If the release asset does not exist yet, publish it once from a
machine that has the file:

    gh release create $TAG vision/sidewalk_frodobots.pt \\
      --repo $REPO --title "sidewalk policy v1" \\
      --notes "resnet18, 96px, FrodoBots-2K getting-started subset. See vision/MODEL_CARD.md"

Or skip the download entirely and train your own: vision/colab_frodobots.ipynb
EOF
  exit 1
fi

verify
echo "OK -> $OUT"
