"""Headless replay of one batch trial with follower/planner instrumentation.

Usage: python repro_trial.py <object_idx> <batch_dir>
"""
import contextlib
import io
import json
import sys
from pathlib import Path

import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

_buf = io.StringIO()
with contextlib.redirect_stdout(_buf):
    from pick_nav_reach_env import PickNavReachEnv
    from solver import Solver
    from custom_libraries.grasp import GraspFollower, GraspPlanner

object_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 7
batch = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(
    'runs/20261111_205416_66b6bb65')
spec = json.loads((batch / 'maze.json').read_text())
maze_path = batch / 'maze.urdf'

object_path = sorted(Path('assets/ycb_objects').glob('*'))[object_idx]
print("OBJECT", object_path.name)


class FixedMazeEnv(PickNavReachEnv):
    def _load_maze(self):
        self.maze_out_pos_x, self.maze_out_pos_y = spec['goal_xy']
        self.cube_positions = np.asarray(spec['cube_positions'], dtype=float)
        p.loadURDF(str(maze_path), useFixedBase=True,
                   flags=p.URDF_MERGE_FIXED_LINKS,
                   physicsClientId=self.pb_physics_client)


CONTACT_LOG = []
orig_contacts = GraspFollower._contacts


def logging_contacts(self):
    owner = self.planner
    contacts = p.getContactPoints(owner.robot, owner.object,
                                  physicsClientId=owner.client)
    pos = [c for c in contacts if c[9] > 0.01]
    fset = {c[3] for c in pos}
    ok = set(owner.finger_ids).issubset(fset)
    if len(CONTACT_LOG) < 2000:
        CONTACT_LOG.append(dict(
            phase=self.plan.phases[self.index], idx=self.index,
            n=len(contacts), n_pos=len(pos),
            links=sorted({owner.info[c[3]][12].decode() if c[3] >= 0
                          else 'base' for c in pos}),
            depths=[round(float(min(c[7][2], c[8])), 4) for c in contacts][:4],
            ok=ok))
    return ok


GraspFollower._contacts = logging_contacts

env = FixedMazeEnv(seed=42, object_idx=object_idx, maze_urdf_path=maze_path,
                   gui=False)
env.reset()
solver = Solver(env)
last_state = None
plan_logged = False
for i in range(1000):
    action = solver.get_action(env)
    if action is None:
        print(f"step {i}: solver returned None (state {solver.state})")
        break
    obs, info = env.step(action)
    if solver.state != last_state:
        print(f">>> step {i}: state {last_state} -> {solver.state}", flush=True)
        last_state = solver.state
        if solver.state == 4:
            break
    if getattr(solver, "grasp_error", None) and not plan_logged:
        plan_logged = True

print("FINAL state:", solver.state, "error:", getattr(solver, "grasp_error", None))
print("stats:", getattr(solver.grasp_planner, "last_stats", None))
if CONTACT_LOG:
    phases = {}
    for rec in CONTACT_LOG:
        phases.setdefault(rec["phase"], []).append(rec)
    for ph, recs in phases.items():
        oks = sum(r["ok"] for r in recs)
        print(f"phase {ph}: {len(recs)} checks, bilateral_ok={oks}, "
              f"typical n={recs[len(recs)//2]['n']} links={recs[len(recs)//2]['links']}")
obj_pos = np.asarray(env._get_obs()["object_pos"])
print("object at:", np.round(obj_pos, 3))
