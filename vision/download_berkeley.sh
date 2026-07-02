#!/usr/bin/env bash
# Download + reconstruct Berkeley-FrodoBots-7K (gated; run `hf auth login` first).
# It's a Zarr store split into 24 tar.gz parts (~769 GB total, ~1 TB peak with the
# extracted copy). Run this on a big-disk box (cloud GPU instance), NOT a laptop.
set -euo pipefail

REPO="BitRobot/Berkeley-FrodoBots-7K"
DEST="${1:-./berkeley7k}"          # target dir (default ./berkeley7k)
mkdir -p "$DEST"

echo ">> downloading 24 parts (~769 GB) to $DEST ..."
hf download "$REPO" --repo-type dataset --local-dir "$DEST" \
  --include "frodobots_dataset.tar.gz.part_*"

echo ">> recombining split archive ..."
cat "$DEST"/frodobots_dataset.tar.gz.part_* > "$DEST/frodobots_dataset.tar.gz"

echo ">> extracting zarr store (this is large) ..."
tar -xzf "$DEST/frodobots_dataset.tar.gz" -C "$DEST"

echo ">> done. Zarr store at: $DEST/frodobots_dataset/dataset_cache.zarr"
echo ">> inspect it:  PYTHONPATH=vision python3 -c \"import zarr; z=zarr.open('$DEST/frodobots_dataset/dataset_cache.zarr','r'); print(z.tree())\""
