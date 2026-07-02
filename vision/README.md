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
### Option A — Berkeley-FrodoBots-7K (recommended: LeRobot format, reannotated labels)
1. Accept terms: https://huggingface.co/datasets/frodobots/Berkeley-FrodoBots-7K (gated).
2. `.venv/bin/huggingface-cli login` (or `export HF_TOKEN=...`).
3. `PYTHONPATH=vision .venv/bin/python inspect_dataset.py` — confirm the `action` shape.
4. Use `action_mbra` (Model-Based Reannotation — cleaner). It's a **nav action** (waypoint
   chunk / velocity), so set `action_dim` to match and convert the first waypoint→heading
   into `angular` for the rover (atan2 of the relative waypoint), or use its yaw directly.

### Option B — FrodoBots-2K (raw teleop, maps 1:1 to /control)
- `huggingface_hub.snapshot_download` a few ride ids (front MP4 20fps 1024×576 + 10 Hz
  control CSV). Fill `FrodoBots2KDataset._build`: parse control (linear/angular; derive
  ω from the 4 wheel RPMs if needed), index frames at a stride, align by timestamp.
- Target = `(linear, angular)` — already the rover's control space, no conversion.

## Train real → deploy
```bash
# swap SyntheticSidewalkDataset -> your real Dataset in train.py, then:
PYTHONPATH=vision .venv/bin/python train.py --epochs 30 --backbone resnet18
```
Use MPS for a subset now; a cloud GPU (A10/A100) for the full set. Then in the live loop:
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
