"""Per-trial grasp-phase base behaviour.

Park frame = last frame (after object landed) where the base was still
moving. From park to end: base drift, object z rise, finger closure,
and how it ended.
"""
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
        print(f"{trial:42s} EMPTY")
        continue
    qpos = np.array([r['qpos'] for r in recs])
    act = np.array([r['action'] for r in recs])
    objs = np.array([r['object_pos'] for r in recs])
    base = qpos[:, 0:2]
    vel = np.r_[[0.0], np.linalg.norm(np.diff(base, axis=0), axis=1)]
    landed = np.nonzero(objs[:, 2] < 1.0)[0]
    if len(landed) == 0:
        print(f"{trial:42s} object never landed")
        continue
    t0 = landed[0]
    moving = np.nonzero(vel[t0:] > 0.02)[0]
    park = t0 + (moving[-1] if len(moving) else 0)
    seg_base = base[park:]
    seg_obj = objs[park:]
    seg_fing = act[park:, 13:15]
    seg_arm_err = np.abs(np.diff(qpos[park:, 6:13], axis=0)).max(axis=1) \
        if len(qpos) > park + 1 else np.zeros(1)
    drift = np.linalg.norm(seg_base[-1] - seg_base[0])
    z_rise = seg_obj[:, 2].max() - seg_obj[0, 2]
    fing_min = seg_fing.min() if len(seg_fing) else 1.0
    # object knock-over / slide before any closure
    xy_slide = np.linalg.norm(seg_obj[:, :2] - seg_obj[0, :2], axis=1)
    print(f"{trial:42s} steps={len(recs):4d} park@{park:4d} "
          f"base_drift_after_park={drift:.3f} obj_z_rise={z_rise:+.3f} "
          f"obj_xy_slide={xy_slide.max():.3f} finger_cmd_min={fing_min:.4f}")
