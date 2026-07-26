# Vision — sidewalk-keeping (Urban track)

Turns "reaches GPS checkpoints" into "stays on the sidewalk between them." A behavior-
cloning policy maps the front camera frame → steering, fused with the GPS-bearing
follower: **GPS picks the direction, vision keeps the wheels on the walkable surface.**

> The trained checkpoint is **not in the repo** (43 MB, `*.pt` is gitignored).
> `bash vision/fetch_model.sh` downloads and verifies it; `MODEL_CARD.md` says what it is,
> what it was trained on, and why you should gate it hard. GPS-only navigation needs none of it.
>
> Checkpoints are loaded with `weights_only=True`: a `.pt` is a pickle, and loading one with
> `weights_only=False` executes whatever is inside it — which matters more now that a script
> downloads one. Keep any new loader on the safe path, and keep `train.py` saving only tensors
> and primitives so it stays possible.

## Files
| File | Role |
|---|---|
| `policy.py` | `SidewalkPolicy` — CNN (`tiny` or `resnet18`) → (linear, angular). |
| `dataset.py` | `SyntheticSidewalkDataset` (runs now) + `FrodoBots2KDataset` (real, stub to fill). |
| `train.py` | BC trainer (MSE, MPS/CUDA). |
| `fuse.py` | `fuse_steer(gps, vision)` → `/control` + end-to-end demo. |
| `inspect_dataset.py` | Stream a real sample once you have HF access. |
| `fetch_model.sh` | Download + sha256-verify the default `--vision` checkpoint. |
| `MODEL_CARD.md` | What the shipped checkpoint is, measured behaviour, and its limits. |

## Proven now (synthetic, no data/GPU needed)
```bash
PYTHONPATH=vision .venv/bin/python train.py --epochs 8      # val_mse -> ~1e-4, steer_sign_acc -> 1.00
PYTHONPATH=vision .venv/bin/python fuse.py                  # policy + fusion end-to-end
```
The synthetic task (steer toward a visible path band) is a real steer-from-pixels signal —
fitting it validates the architecture, training, inference, and fusion. It's a weak prior,
**not** the competition policy; that needs the real data below.

> Gotcha already fixed: the backbone pools over height but keeps width
> (`AdaptiveAvgPool2d((1,8))`) — global pooling destroys the horizontal position that
> steering depends on (mean-prediction, sign-acc stuck at 0.5).

## Unlock real data (you own the HF steps)
### Option A — Berkeley-FrodoBots-7K (recommended: reannotated MBRA labels)
Repo is now **`BitRobot/Berkeley-FrodoBots-7K`** (gated). It is a **Zarr store split across 24
`tar.gz` parts, ~769 GB** (~1 TB peak extracted) — NOT tabular; streaming yields raw zarr
chunks. So this is a **cloud/big-disk job**, not a laptop one.
1. Accept terms in the browser + `hf auth login` (done ✓ for Aiden).
2. On a box with ~1 TB disk + GPU: `bash vision/download_berkeley.sh ./berkeley7k`
   (downloads the 24 parts, `cat`s them, extracts the zarr).
3. `python3 vision/inspect_zarr.py ./berkeley7k/frodobots_dataset/dataset_cache.zarr`
   — prints the real array shapes (`action`, `action_mbra`, image arrays/paths).
4. Write the Dataset against those shapes: yield `(front frame, action_mbra[i])`, set
   `SidewalkPolicy(action_dim=<last dim of action_mbra>)`. `action_mbra` is a nav action
   (waypoint chunk / velocity) — convert its first waypoint→heading to `angular` for the
   rover (`atan2` of the relative waypoint), or use its yaw. The MBRA paper's `frodo-vla`
   repo is the canonical reference loader.

### Option B: FrodoBots-2K (raw teleop, maps 1:1 to /control) — IMPLEMENTED
`FrodoBots2KDataset` in `dataset.py` is done and verified on real rides. Access is via 24
zip parts listed in `complete-dataset.csv` on S3 (NOT per-ride files):
- **Quick local check** (~343 MB, a few rides):
  ```bash
  curl -o gs.zip https://frodobots-2k-dataset.s3.ap-southeast-1.amazonaws.com/frodobots-dataset-getting-started.zip
  unzip gs.zip -d data
  PYTHONPATH=vision python train.py --data data/frodobots-dataset-getting-started \
    --backbone resnet18 --img 96 --stride 8 --epochs 15 --out sidewalk_frodobots.pt
  ```
- **Full run on Colab:** `vision/colab_frodobots.ipynb` downloads one ~19 GB part and trains.

Each ride has `control_data_<id>.csv` (columns `linear, angular, rpm_1..4, timestamp`) and
`front_camera_timestamps_<id>.csv` (`frame_id, timestamp`). The loader aligns each sampled
frame to the nearest control row by timestamp; target `(linear, angular)` is already the
rover's control space (the `rpm_*` wheel columns are there if you want to derive it, but not
needed). Idle frames are dropped by default; frames are decoded once and cached as JPEGs.

## Train real → deploy
```bash
# real data is just the --data flag now (no code edit):
PYTHONPATH=vision python train.py --data <frodobots_root> \
  --backbone resnet18 --img 96 --stride 6 --epochs 30 --out sidewalk_frodobots.pt
```
Use MPS for a subset now; a cloud GPU (A10/A100) or Colab for the full set. Then in the live loop:
```python
frame, _ = client.get_front_frame()          # jpeg bytes from /v2/front
img = preprocess(frame, size=ck["img"])       # decode -> resize -> /255 -> CHW tensor
v_lin, v_ang = policy.act(img)
lin, ang = fuse_steer(gps_angular, v_ang, gps_linear, v_lin)   # gps_* from waypoint_follower
client.control(lin, ang)
```
(`preprocess` = one small helper: `PIL.Image.open(BytesIO(frame)).resize((s,s))` → tensor/255.)

## Obstacle stop (`--blocked`) — plumbing done, model NOT trustworthy yet

`SidewalkPolicy(blocked_head=True)` adds a second output, P(path blocked), trained with BCE
against a weak label from teleop: *the human was moving and has just stopped*
(`blocked.blocked_labels`). There is no obstacle annotation in FrodoBots-2K, and a brake event
is the closest real signal there is — it also fires for red lights, hesitation and boredom.

```bash
PYTHONPATH=vision python train.py --data data/frodobots-dataset-getting-started \
  --backbone resnet18 --img 96 --stride 4 --epochs 8 --blocked --out sidewalk_blocked.pt
```

Measured on the getting-started subset (3 rides, 3983 samples, 398 val):

```
epoch  1-5   blocked[acc=0.98 p=0.00 r=0.00 base=0.02]   <- predicts "never blocked"
epoch  7     blocked[acc=0.98 p=1.00 r=0.33 base=0.02]
epoch  8     blocked[acc=0.96 p=0.37 r=0.78 base=0.02]
```

**Read that as a negative result.** Positives are 2% of the data — about 9 examples in the
validation split — so precision/recall swing wildly between epochs and none of it is
significant. The 0.98 accuracy is just the base rate. For five epochs the model simply learned
to say "never blocked", which is exactly the failure this head exists to prevent.

What it does show: the label, the head, the loss, the metrics and the follower's gate all work
end to end. What it needs: the full dataset, and ideally real obstacle annotation rather than a
braking proxy.

Until then no shipped checkpoint has the head, so `p` is `None` and the gate never brakes.

## Roadmap
- Train the stop head on the full dataset; treat the numbers above as a floor, not a result.
- Recovery: on repeated low-confidence frames, back up + re-scan (ties to the SDK
  interventions API).
- Off-road / Indoor tracks: image-goal policy (no GPS) — same backbone, goal-image input.
