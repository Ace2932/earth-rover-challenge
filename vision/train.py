"""Behavior-cloning trainer for the sidewalk policy.

Proves the pipeline on the synthetic task (loss should fall sharply and the model
should steer toward the band on held-out samples). Swap SyntheticSidewalkDataset for
FrodoBots2KDataset to train on real data — same loop.

  python3 vision/train.py --epochs 6 --backbone tiny
"""
import argparse
import torch
from torch.utils.data import DataLoader

from policy import SidewalkPolicy
from dataset import SyntheticSidewalkDataset, FrodoBots2KDataset, find_ride_dirs


def device():
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def evaluate(model, ds, dev, n=512):
    dl = DataLoader(ds, batch_size=256)
    model.eval()
    se, cnt, sign_ok = 0.0, 0, 0
    with torch.no_grad():
        for img, tgt in dl:
            img, tgt = img.to(dev), tgt.to(dev)
            out = model(img)
            se += torch.nn.functional.mse_loss(out, tgt, reduction="sum").item()
            cnt += tgt.numel()
            # does predicted angular turn the correct way vs target angular?
            sign_ok += ((out[:, 1] * tgt[:, 1]) >= 0).sum().item()
            if cnt >= n * tgt.shape[1]:
                break
    return se / cnt, sign_ok / (cnt // tgt.shape[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=6)
    ap.add_argument("--backbone", default="tiny", choices=["tiny", "resnet18"])
    ap.add_argument("--img", type=int, default=64)
    ap.add_argument("--out", default="vision/sidewalk_policy.pt")
    ap.add_argument("--data", default=None,
                    help="FrodoBots-2K root (dir of ride_<id>/). Omit for synthetic.")
    ap.add_argument("--stride", type=int, default=4,
                    help="sample every Nth front-camera frame (real data only)")
    ap.add_argument("--max-rides", type=int, default=None,
                    help="cap number of rides used from --data (first N)")
    args = ap.parse_args()

    dev = device()
    print(f"device={dev} backbone={args.backbone}")
    if args.data:
        rides = find_ride_dirs(args.data) or [args.data]
        if args.max_rides:
            rides = rides[:args.max_rides]
        full = FrodoBots2KDataset(rides, img_size=args.img, stride=args.stride)
        n_val = max(1, int(0.1 * len(full)))
        n_train = len(full) - n_val
        gen = torch.Generator().manual_seed(0)
        train_ds, val_ds = torch.utils.data.random_split(
            full, [n_train, n_val], generator=gen)
        print(f"FrodoBots-2K: {len(full)} samples ({n_train} train / {n_val} val)")
    else:
        train_ds = SyntheticSidewalkDataset(n=4096, img_size=args.img, seed=0)
        val_ds = SyntheticSidewalkDataset(n=512, img_size=args.img, seed=999)
    dl = DataLoader(train_ds, batch_size=128, shuffle=True)

    model = SidewalkPolicy(backbone=args.backbone).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    lossf = torch.nn.MSELoss()

    v0, acc0 = evaluate(model, val_ds, dev)
    print(f"init      val_mse={v0:.4f}  steer_sign_acc={acc0:.2f}")
    for ep in range(args.epochs):
        model.train()
        tot = 0.0
        for img, tgt in dl:
            img, tgt = img.to(dev), tgt.to(dev)
            opt.zero_grad()
            loss = lossf(model(img), tgt)
            loss.backward()
            opt.step()
            tot += loss.item() * img.size(0)
        vmse, acc = evaluate(model, val_ds, dev)
        print(f"epoch {ep+1:2d}  train_mse={tot/len(train_ds):.4f}  "
              f"val_mse={vmse:.4f}  steer_sign_acc={acc:.2f}")

    torch.save({"state_dict": model.state_dict(), "backbone": args.backbone,
                "img": args.img}, args.out)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
