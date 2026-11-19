"""Per-trial outcome detail for the delayed batch.

After park: did the arm move (plan found?), did fingers close, contacts,
final arm pose (extended=crashed in state-4 hold), object z at end.
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
        print(f"{trial:40s} EMPTY")
        continue
    qpos = np.array([r['qpos'] for r in recs])
    objs = np.array([r['object_pos'] for r in recs])
    ncontact = [len(r.get('contacts') or []) for r in recs]
    base = qpos[:, 0:2]
    vel = np.r_[[0.0], np.linalg.norm(np.diff(base, axis=0), axis=1)]
    landed = np.nonzero(objs[:, 2] < 1.0)[0]
    t0 = landed[0] if len(landed) else 0
    moving = np.nonzero(vel[t0:] > 0.02)[0]
    park = t0 + (moving[-1] if len(moving) else 0)
    arm = qpos[park:, 6:13]
    fing = qpos[park:, 13:15]
    arm_range = (arm.max(axis=0) - arm.min(axis=0)).max()
    fing_open = fing.max(axis=0).max()
    fing_close = fing.min(axis=0).min()
    contacts_peak = max(ncontact[park:]) if park < len(ncontact) else 0
    # how long after park did the arm finish moving
    arm_move = np.r_[[0.0], np.linalg.norm(np.diff(arm, axis=0), axis=1)]
    last_move = np.nonzero(arm_move > 0.01)[0]
    end_hold = 'extended-arm-hold' if last_move.size and (
        len(arm) - last_move[-1]) > 100 else 'still-moving-or-short'
    print(f"{trial:40s} park@{park:3d} end={len(recs):4d} "
          f"arm_range={arm_range:.2f} fing:{fing_close:.3f}->{fing_open:.3f} "
          f"contacts_peak={contacts_peak:2d} obj_end=({objs[-1][0]:.2f},{objs[-1][1]:.2f},{objs[-1][2]:.3f}) "
          f"last_arm_move@+{last_move[-1] if last_move.size else 0}")
