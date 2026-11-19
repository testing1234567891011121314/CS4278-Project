import glob
import json
import numpy as np

for f in sorted(glob.glob('runs/20261111_205416_66b6bb65/*/*.jsonl')):
    trial = f.split('\\')[-2]
    recs = []
    for line in open(f, encoding='utf-8', errors='replace'):
        try:
            recs.append(json.loads(line))
        except Exception:
            pass
    if not recs:
        continue
    qpos = np.array([r['qpos'] for r in recs])
    fing = qpos[:, 13:15]
    # post-200-wait finger behaviour
    fing_post = fing[200:]
    t_open = fing_post.argmax(axis=0).max()
    print(f"{trial:40s} fing_open={fing_post.max(axis=0).max():.4f} "
          f"at+{t_open}  fing_end={fing_post[-1].round(4).tolist()} "
          f"min_after_open={fing_post[t_open:, :].min():.4f}")
