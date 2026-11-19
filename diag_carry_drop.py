"""Bounded replay of the best carry run: run apple/seed-42 to the grasp,
then carry, dumping per-step contact + pose telemetry near the fall so we can
tell whether the apple slipped from the fingertips (no grip contact) or was
pushed out by a collision (fingertip/object/world contact)."""
import json
from pathlib import Path
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

run = Path("runs/20261113_072204_61c61196")
spec = json.loads((run / "maze.json").read_text(encoding="utf-8"))
maze_path = Path(spec["urdf_path"])


class FixedMazeEnv(PickNavReachEnv):
    def _load_maze(self):
        self.maze_out_pos_x, self.maze_out_pos_y = spec["goal_xy"]
        self.cube_positions = np.asarray(spec["cube_positions"], dtype=float)
        p.loadURDF(str(maze_path), useFixedBase=True,
                   flags=p.URDF_MERGE_FIXED_LINKS,
                   physicsClientId=self.pb_physics_client)


env = FixedMazeEnv(seed=42, object_idx=7, gui=False)
env.reset()
solver = Solver(env)

FINGER = {21: 'r_finger', 22: 'l_finger'}
cid = env.pb_physics_client


def contacts_with_object():
    obj = env.object_id
    out = []
    for ct in p.getContactPoints(bodyA=obj, physicsClientId=cid):
        # ct: (0, bodyA, bodyB, linkA, linkB, posA, posB, normal, dist, ...)
        other, link = ct[2], ct[4]
        who = FINGER.get(link, f'body{other}/link{link}')
        out.append((who, round(ct[8], 5)))
    return out


steps = 0
fell = False
while steps < 1500 and not fell:
    obs = env._get_obs()
    q = np.asarray(obs["qpos"], float)
    obj_pos = np.asarray(obs["object_pos"], float)
    solver.get_action(env)
    env.step(np.asarray(solver.action, dtype=float))
    steps += 1
    after = env._get_obs()
    aq = np.asarray(after["qpos"], float)
    aobj = np.asarray(after["object_pos"], float)
    base_xy = np.array([aq[0] - 2.0, aq[1]])
    if solver.state == 3 or (solver._carry_stage if hasattr(solver, "_carry_stage") else None) is not None:
        # in carry: log every step in the last 40 before fall
        d_base_obj = float(np.linalg.norm(aobj[:2] - base_xy))
        if aobj[2] < 0.30 or d_base_obj > 1.2:
            fell = True
            print(f"FALL at step {steps}", flush=True)
            print("  obj z:", round(float(aobj[2]), 4),
                  "obj xy:", np.round(aobj[:2], 3),
                  "base xy:", np.round(base_xy, 3),
                  "d:", round(d_base_obj, 3), flush=True)
            print("  contacts:", contacts_with_object(), flush=True)
            print("  r_finger:", round(float(aq[13]), 5),
                  "l_finger:", round(float(aq[14]), 5),
                  "fingers cmd:", np.round(solver.action[13:15], 5), flush=True)
            break
        # dump carry telemetry whenever the apple gets low or drifts far
        if getattr(solver, "_carry_stage", None) is not None:
            c = contacts_with_object()
            if (not c and d_base_obj > 0.75) or aobj[2] < 0.60 \
                    or d_base_obj > 0.95:
                print(f"s{steps} obj_z={aobj[2]:.4f} d={d_base_obj:.3f} "
                      f"obj=({aobj[0]:.3f},{aobj[1]:.3f}) "
                      f"rf={aq[13]:.5f} lf={aq[14]:.5f} "
                      f"cmd_f={solver.action[13]:.5f},{solver.action[14]:.5f} "
                      f"base=({base_xy[0]:.3f},{base_xy[1]:.3f}) "
                      f"yaw={aq[2]:.2f} ct={c}", flush=True)
print("done at", steps, "fell:", fell, "state:", solver.state, flush=True)
