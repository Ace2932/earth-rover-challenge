"""Stream a few real frames from Berkeley-FrodoBots-7K to learn the action schema.
No full download — streaming=True pulls only what we read."""
import sys
from datasets import load_dataset

REPO = "frodobots/Berkeley-FrodoBots-7K"
try:
    ds = load_dataset(REPO, split="train", streaming=True)
except Exception as e:
    print("LOAD ERROR:", type(e).__name__, str(e)[:300])
    print("\nIf this is a gating/401 error, accept terms at https://huggingface.co/datasets/%s" % REPO)
    print("then `huggingface-cli login` (or set HF_TOKEN).")
    sys.exit(1)

it = iter(ds)
for n in range(2):
    try:
        ex = next(it)
    except Exception as e:
        print("ITER ERROR:", type(e).__name__, str(e)[:300]); sys.exit(1)
    print(f"\n=== example {n} — keys ===")
    for k, v in ex.items():
        t = type(v).__name__
        info = ""
        try:
            if hasattr(v, "shape"): info = f"shape={tuple(v.shape)}"
            elif isinstance(v, (list, tuple)): info = f"len={len(v)} sample={v[:6]}"
            elif isinstance(v, (int, float, str, bool)) or v is None: info = f"val={v}"
            else: info = f"repr={repr(v)[:80]}"
        except Exception as ex2:
            info = f"<{ex2}>"
        print(f"  {k:42s} {t:12s} {info}")
