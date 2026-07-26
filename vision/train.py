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
        for batch in dl:
            img, tgt = batch[0].to(dev), batch[1].to(dev)
            out = model(img)
            if isinstance(out, tuple):
                out = out[0]
            se += torch.nn.functional.mse_loss(out, tgt, reduction="sum").item()
            cnt += tgt.numel()
            # does predicted angular turn the correct way vs target angular?
            sign_ok += ((out[:, 1] * tgt[:, 1]) >= 0).sum().item()
            if cnt >= n * tgt.shape[1]:
                break
    return se / cnt, sign_ok / (cnt // tgt.shape[1])


def evaluate_blocked(model, ds, dev):
    """Held-out metrics for the stop head. Accuracy is close to useless on an
    imbalanced label, so report precision/recall at 0.5 as well: a missed stop and
    a false stop cost completely different things."""
    dl = DataLoader(ds, batch_size=256)
    model.eval()
    tp = fp = fn = tn = 0
    with torch.no_grad():
        for batch in dl:
            if len(batch) < 3:
                return None
            img, blk = batch[0].to(dev), batch[2].to(dev)
            out = model(img)
            if not isinstance(out, tuple):
                return None
            p = torch.sigmoid(out[1].squeeze(-1))
            pred, truth = p >= 0.5, blk >= 0.5
            tp += (pred & truth).sum().item()
            fp += (pred & ~truth).sum().item()
            fn += (~pred & truth).sum().item()
            tn += (~pred & ~truth).sum().item()
    total = tp + fp + fn + tn
    return {"positives": (tp + fn) / max(1, total),
            "accuracy": (tp + tn) / max(1, total),
            "precision": tp / max(1, tp + fp),
            "recall": tp / max(1, tp + fn)}


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
    ap.add_argument("--blocked", action="store_true",
                    help="also train the stop head on the weak 'human braked here' "
                         "label (real data only)")
    ap.add_argument("--blocked-weight", type=float, default=1.0)
    args = ap.parse_args()

    dev = device()
    print(f"device={dev} backbone={args.backbone}")
    if args.data:
        rides = find_ride_dirs(args.data) or [args.data]
        if args.max_rides:
            rides = rides[:args.max_rides]
        full = FrodoBots2KDataset(rides, img_size=args.img, stride=args.stride,
                                  with_blocked=args.blocked)
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

    if args.blocked and not args.data:
        raise SystemExit("--blocked needs real data (--data); the synthetic task has no "
                         "notion of an obstacle")
    model = SidewalkPolicy(backbone=args.backbone, blocked_head=args.blocked).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    lossf = torch.nn.MSELoss()
    bce = torch.nn.BCEWithLogitsLoss()

    v0, acc0 = evaluate(model, val_ds, dev)
    print(f"init      val_mse={v0:.4f}  steer_sign_acc={acc0:.2f}")
    for ep in range(args.epochs):
        model.train()
        tot = 0.0
        for batch in dl:
            img, tgt = batch[0].to(dev), batch[1].to(dev)
            opt.zero_grad()
            out = model(img)
            if isinstance(out, tuple):
                action, logit = out
                loss = lossf(action, tgt) + args.blocked_weight * bce(
                    logit.squeeze(-1), batch[2].to(dev))
            else:
                loss = lossf(out, tgt)
            loss.backward()
            opt.step()
            tot += loss.item() * img.size(0)
        vmse, acc = evaluate(model, val_ds, dev)
        line = (f"epoch {ep+1:2d}  train_loss={tot/len(train_ds):.4f}  "
                f"val_mse={vmse:.4f}  steer_sign_acc={acc:.2f}")
        b = evaluate_blocked(model, val_ds, dev) if args.blocked else None
        if b:
            line += (f"  blocked[acc={b['accuracy']:.2f} p={b['precision']:.2f} "
                     f"r={b['recall']:.2f} base={b['positives']:.2f}]")
        print(line)

    torch.save({"state_dict": model.state_dict(), "backbone": args.backbone,
                "img": args.img, "blocked_head": args.blocked}, args.out)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
