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


class FrodoBots2KDataset(Dataset):
    """Real teleop data. Downloads a few ride files from HF, decodes front-cam frames,
    aligns to the 10 Hz control stream -> (frame, [linear, angular]).

    Requires: `huggingface-cli login` (rate limits) and torchvision video decode.
    Left as an explicit loader you run once you've picked ride ids; see vision/README.md.
    """
    def __init__(self, ride_files, img_size=64, stride=4):
        self.samples = []       # list of (mp4_path, frame_idx, action)
        self.img_size = img_size
        self._build(ride_files, stride)

    def _build(self, ride_files, stride):
        # Intentionally minimal: expects locally-downloaded (front_mp4, control_csv) pairs.
        # Parse control CSV (10 Hz: timestamp, linear/angular or wheel RPM -> v,omega),
        # index video frames at `stride`, store (path, frame_idx, action).
        # Kept as a stub so this module imports without the ~GB downloads; fill in with
        # your chosen ride ids per vision/README.md ("Unlock real data").
        raise NotImplementedError(
            "Download rides then implement CSV<->frame alignment — see vision/README.md")
