"""Datasets for sidewalk-keeping behavior cloning.

SyntheticSidewalkDataset — no download, runs now. Renders a front-camera-ish frame
with a lighter "sidewalk" band at a random horizontal offset; the supervised target
steers toward centering the band. A model that fits this has genuinely learned
steer-from-pixels — it proves the whole pipeline and is a usable weak prior.

FrodoBots2KDataset — the real thing (raw MP4 + 10 Hz control). Wired but needs the
HF files (see load()); imports lazily so this module runs without them.
Target = teleop (linear, angular), which maps directly onto the rover's /control.
"""
import math
import torch
from torch.utils.data import Dataset


class SyntheticSidewalkDataset(Dataset):
    def __init__(self, n=4096, img_size=64, seed=0, noise=0.05):
        self.n = n
        self.s = img_size
        self.noise = noise
        self.seed = seed

    def __len__(self):
        return self.n

    def __getitem__(self, i):
        g = torch.Generator().manual_seed(self.seed * 1_000_003 + i)
        S = self.s
        img = torch.full((3, S, S), 0.35)                      # background (road/grass)
        band_w = int(S * 0.30)
        # band center in [band_w/2 .. S-band_w/2]
        lo, hi = band_w // 2, S - band_w // 2
        c = int(torch.randint(lo, hi + 1, (1,), generator=g))
        x0, x1 = max(0, c - band_w // 2), min(S, c + band_w // 2)
        img[:, :, x0:x1] = 0.75                                # lighter sidewalk band
        img += self.noise * torch.randn(3, S, S, generator=g)
        img = img.clamp(0, 1)
        # supervision: turn toward centering the band; slow if band near an edge
        c_norm = (c - S / 2) / (S / 2)                          # -1 (left) .. +1 (right)
        angular = 0.8 * c_norm
        edge = abs(c_norm)
        linear = 0.6 * (1.0 - 0.5 * edge)                       # ease off when band is off-center
        return img, torch.tensor([linear, angular], dtype=torch.float32)


def find_ride_dirs(root):
    """Return sorted ride directories (.../ride_<id>_<ts>/ with a control CSV) under root."""
    import os
    out = []
    for d, _dirs, files in os.walk(root):
        if os.path.basename(d).startswith("ride_") \
                and any(f.startswith("control_data_") for f in files):
            out.append(d)
    return sorted(out)


def _ride_id(ride_dir):
    import os
    # ride_<id>_<timestamp> -> <id>
    return os.path.basename(ride_dir).split("_")[1]


class FrodoBots2KDataset(Dataset):
    """Real FrodoBots-2K teleop data -> (front frame, [linear, angular]).

    Each ride dir holds control_data_<id>.csv (10 Hz: linear, angular, rpm_1..4,
    timestamp), front_camera_<id>.mp4 (20 fps, 1024x576) and
    front_camera_timestamps_<id>.csv (frame_id, timestamp). We align each sampled
    frame to the nearest control row by timestamp; the target (linear, angular) is
    already the rover's /control space, so no conversion is needed.

    Frames are decoded once in a single sequential pass per ride and cached as small
    JPEGs (no fragile per-item random seeking), so epochs after the first are fast.

    ride_dirs: a ride dir, list of ride dirs, or a dataset root (auto-discovered).
    """
    def __init__(self, ride_dirs, img_size=96, stride=4, cache_dir=None,
                 drop_idle=True, max_frames_per_ride=None, tolerance=0.2):
        self.img_size = img_size
        self.samples = []       # list of (jpg_path, action float32[2])
        self._build(ride_dirs, stride, cache_dir, drop_idle,
                    max_frames_per_ride, tolerance)

    def _build(self, ride_dirs, stride, cache_dir, drop_idle,
               max_frames_per_ride, tolerance):
        import os
        import cv2
        import numpy as np
        import pandas as pd

        if isinstance(ride_dirs, str):
            found = find_ride_dirs(ride_dirs)
            ride_dirs = found if found else [ride_dirs]

        for ride_dir in ride_dirs:
            rid = _ride_id(ride_dir)
            mp4 = os.path.join(ride_dir, f"front_camera_{rid}.mp4")
            ctrl_p = os.path.join(ride_dir, f"control_data_{rid}.csv")
            ts_p = os.path.join(ride_dir, f"front_camera_timestamps_{rid}.csv")
            if not (os.path.exists(mp4) and os.path.exists(ctrl_p) and os.path.exists(ts_p)):
                continue

            ctrl = pd.read_csv(ctrl_p)[["timestamp", "linear", "angular"]] \
                .sort_values("timestamp").reset_index(drop=True)
            ts = pd.read_csv(ts_p)[["frame_id", "timestamp"]] \
                .sort_values("timestamp").reset_index(drop=True)
            # nearest control row within `tolerance` seconds of each frame
            merged = pd.merge_asof(ts, ctrl, on="timestamp",
                                   direction="nearest", tolerance=tolerance) \
                .dropna(subset=["linear", "angular"])
            merged = merged.iloc[::stride]
            if drop_idle:
                moving = (merged["linear"].abs() > 0.05) | (merged["angular"].abs() > 0.05)
                merged = merged[moving]
            if max_frames_per_ride:
                merged = merged.iloc[:max_frames_per_ride]

            wanted = {int(r.frame_id): (float(r.linear), float(r.angular))
                      for r in merged.itertuples()}
            if not wanted:
                continue

            cache = cache_dir or (ride_dir.rstrip("/") + "_frames")
            os.makedirs(cache, exist_ok=True)
            cap = cv2.VideoCapture(mp4)
            idx = 0
            ok, frame = cap.read()
            while ok:
                if idx in wanted:
                    jpg = os.path.join(cache, f"{rid}_{idx:06d}_{self.img_size}.jpg")
                    if not os.path.exists(jpg):
                        cv2.imwrite(jpg, cv2.resize(frame, (self.img_size, self.img_size)))
                    lin, ang = wanted[idx]
                    self.samples.append((jpg, np.array([lin, ang], dtype=np.float32)))
                idx += 1
                ok, frame = cap.read()
            cap.release()

        if not self.samples:
            raise RuntimeError(
                "FrodoBots2KDataset: built 0 samples. Check the dataset root has "
                "ride_<id>/ dirs with control_data + front_camera + timestamps.")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, i):
        import cv2
        import torch as _torch
        jpg, action = self.samples[i]
        img = cv2.cvtColor(cv2.imread(jpg), cv2.COLOR_BGR2RGB)
        t = _torch.from_numpy(img).float().permute(2, 0, 1) / 255.0
        return t, _torch.from_numpy(action)
