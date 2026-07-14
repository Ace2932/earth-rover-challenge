# Vision — sidewalk-keeping (Urban track)

Turns "reaches GPS checkpoints" into "stays on the sidewalk between them." A behavior-
cloning policy maps the front camera frame → steering, fused with the GPS-bearing
follower: **GPS picks the direction, vision keeps the wheels on the walkable surface.**

## Files
| File | Role |
|---|---|
| `policy.py` | `SidewalkPolicy` — CNN (`tiny` or `resnet18`) → (linear, angular). |
| `dataset.py` | `SyntheticSidewalkDataset` (runs now) + `FrodoBots2KDataset` (real, stub to fill). |
| `train.py` | BC trainer (MSE, MPS/CUDA). |
| `fuse.py` | `fuse_steer(gps, vision)` → `/control` + end-to-end demo. |
| `inspect_dataset.py` | Stream a real sample once you have HF access. |

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

## Roadmap
- Edge/obstacle stop: add a "no drivable surface ahead" head → force linear→0.
- Recovery: on repeated low-confidence frames, back up + re-scan (ties to the SDK
  interventions API).
- Off-road / Indoor tracks: image-goal policy (no GPS) — same backbone, goal-image input.
