"""Inspect the reconstructed Berkeley-FrodoBots-7K zarr store.

Run this ONCE on the box where you've reconstructed the store (see download_berkeley.sh).
It prints the array tree + each array's shape/dtype + a sample action row, which tells us
the exact `action` / `action_mbra` dimensionality and how frames are stored — everything
needed to write the real Dataset (and set the policy's action_dim) with zero guessing.

    python3 vision/inspect_zarr.py ./berkeley7k/frodobots_dataset/dataset_cache.zarr
"""
import sys
import zarr


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "./berkeley7k/frodobots_dataset/dataset_cache.zarr"
    z = zarr.open(path, mode="r")
    print("=== tree ===")
    try:
        print(z.tree())
    except Exception:
        pass
    print("\n=== arrays ===")

    def walk(g, prefix=""):
        for name in g:
            obj = g[name]
            full = f"{prefix}{name}"
            if hasattr(obj, "shape"):
                info = f"shape={obj.shape} dtype={obj.dtype} chunks={getattr(obj,'chunks',None)}"
                print(f"  {full:45s} {info}")
                if "action" in name.lower():
                    try:
                        print(f"      sample[0] = {obj[0]!r}")
                    except Exception as e:
                        print(f"      (sample read failed: {e})")
            else:
                walk(obj, prefix=full + "/")

    walk(z)
    print("\nNext: set SidewalkPolicy(action_dim=<last dim of action_mbra>), and in the "
          "Dataset yield (front frame, action_mbra[i]). If frames are stored as mp4 paths, "
          "decode with torchvision.io.read_video aligned by frame_index.")


if __name__ == "__main__":
    main()
