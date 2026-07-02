"""Fuse GPS-bearing steering (waypoint_follower) with vision sidewalk-keeping.

The GPS follower knows WHICH WAY the next checkpoint is; the vision policy knows
where the drivable sidewalk is right now. Neither alone is enough on real sidewalks:
GPS-only cuts corners into grass/road; vision-only wanders off toward the wrong goal.

fuse_steer blends them: angular = alpha*gps + (1-alpha)*vision (alpha = how much to
trust the global GPS direction vs. local vision), speed = the more cautious of the
two (so an edge-slowdown from vision always wins). Output plugs straight into
RoverClient.control(linear, angular).

Run `python3 vision/fuse.py` for an end-to-end demo on the trained policy.
"""
import os


def fuse_steer(gps_angular, vision_angular, gps_linear=0.6, vision_linear=0.6, alpha=0.5):
    angular = alpha * gps_angular + (1.0 - alpha) * vision_angular
    angular = max(-1.0, min(1.0, angular))
    linear = max(0.0, min(gps_linear, vision_linear))   # cautious speed
    return linear, angular


def _demo():
    import torch
    from policy import SidewalkPolicy
    from dataset import SyntheticSidewalkDataset

    ckpt_path = "vision/sidewalk_policy.pt"
    if not os.path.exists(ckpt_path):
        print("train first: PYTHONPATH=vision python3 vision/train.py"); return
    ck = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    model = SidewalkPolicy(backbone=ck["backbone"])
    model.load_state_dict(ck["state_dict"])

    # a frame whose sidewalk band sits to the RIGHT -> vision should steer right (+)
    ds = SyntheticSidewalkDataset(n=100, img_size=ck["img"], seed=7)
    # find a right-offset sample
    img, tgt = None, None
    for i in range(100):
        im, t = ds[i]
        if t[1] > 0.4:           # target angular clearly right
            img, tgt = im, t; break
    v_lin, v_ang = model.act(img)
    print(f"vision policy on right-offset frame: linear={v_lin:.2f} angular={v_ang:+.2f} "
          f"(target angular {tgt[1]:+.2f})")

    print("\nfusion cases (alpha=0.5):")
    for name, gps_a, vis_a in [
        ("agree: both steer right", 0.6, v_ang),
        ("GPS straight, sidewalk curves right", 0.0, v_ang),
        ("conflict: GPS right, sidewalk left", 0.6, -0.6),
    ]:
        lin, ang = fuse_steer(gps_a, vis_a)
        print(f"  {name:38s} -> linear={lin:.2f} angular={ang:+.2f}")


if __name__ == "__main__":
    _demo()
