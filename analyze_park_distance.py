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
    objs = np.array([r['object_pos'] for r in recs])
    base = qpos[:, 0:2] - [2.0, 0.0]
    vel = np.r_[[0.0], np.linalg.norm(np.diff(base, axis=0), axis=1)]
    landed = np.nonzero(objs[:, 2] < 1.0)[0]
    t0 = landed[0] if len(landed) else 0
    moving = np.nonzero(vel[t0:] > 0.02)[0]
    park = t0 + (moving[-1] if len(moving) else 0)
    d_park = np.linalg.norm(base[park] - objs[park][:2])
    yaw = qpos[park, 2]
    # vector base->object in base frame
    dvec = objs[park][:2] - base[park]
    c, s = np.cos(-yaw), np.sin(-yaw)
    local = np.array([c * dvec[0] - s * dvec[1], s * dvec[0] + c * dvec[1]])
    objz = objs[park][2]
    print(f"{trial:40s} park_d={d_park:.3f} yaw={yaw:+.2f} "
          f"obj_in_base=(+{local[0]:.2f},{'+' if local[1]>=0 else ''}{local[1]:.2f}) "
          f"obj_z={objz:.3f}")
