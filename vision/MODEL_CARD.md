# Model card — `sidewalk_frodobots.pt`

The checkpoint `waypoint_follower.py --vision` loads by default. It is **a weak prior, not a
competition policy** — read the limitations before trusting it on a real rover.

## What it is

| | |
|---|---|
| Task | Behaviour cloning: front-camera frame → `(linear, angular)` in the rover's `/control` space |
| Architecture | `SidewalkPolicy`, `resnet18` backbone (ImageNet init), 2-layer head → 2 outputs |
| Input | RGB, resized to **96 × 96**, scaled to 0..1, **no ImageNet mean/std normalisation** (training and inference match — do not add it on one side only) |
| Output | `linear` clamped 0..1, `angular` clamped −1..1 |
| Training data | FrodoBots-2K, **getting-started subset only** (`frodobots-dataset-getting-started.zip`, ~343 MB, 3 rides) |
| Labels | The human teleop `linear`/`angular` from `control_data_<id>.csv`, aligned to each frame by nearest timestamp; idle frames dropped |
| File | 43 MB, `sha256` in `sidewalk_frodobots.sha256` |

## Measured behaviour

Run over cached frames from the training subset (so these are **fit**, not held-out, numbers):

```
200 frames spread across the dataset
  linear   median 0.855   mean 0.820   0% below 0.25
  angular  median -0.035  mean -0.040  64% negative

first 10 frames of each ride (rover stationary)
  linear   median 0.698   10% below 0.25
frames 10-60 of each ride
  linear   median 0.876    0% below 0.25
```

## Limitations — read before driving

- **Three rides of one city.** Nothing here generalises to another site, to night, or to rain.
- **A left bias:** `angular` is negative on 64% of sampled frames. Either the training rides
  genuinely turned left more, or the model has partly collapsed to a constant. Not diagnosed.
- **No held-out evaluation on real data.** The `val_mse ~1e-4` quoted in `vision/README.md` is
  from the *synthetic* task, which only ever proved the pipeline works.
- **No obstacle awareness.** It predicts steering, not safety; it cannot tell you to stop
  (issue #7).
- **No uncertainty output**, so the follower's fusion gate treats confidence as 1.0 and relies
  on the heading-error gate and `VISION_ALPHA` instead (issue #11).

Because of all that, the follower gates it hard by default: ignored above
`VISION_MAX_ERR_DEG`, ignored on stale frames, floored at `VISION_MIN_LINEAR`, and off entirely
unless you pass `--vision`. Start at a low `VISION_ALPHA` on a real bot.

## Retraining

The competitive version needs the full dataset:

```bash
# full FrodoBots-2K part on a cloud GPU or Colab
#   vision/colab_frodobots.ipynb
PYTHONPATH=vision python train.py --data <frodobots_root> \
  --backbone resnet18 --img 96 --stride 6 --epochs 30 --out sidewalk_frodobots.pt
```

After retraining, update `sidewalk_frodobots.sha256`, this card, and the release asset
(`bash vision/fetch_model.sh` prints the `gh release create` command).
