"""Replay the batch (FixedMazeEnv + stored maze); at plan time dump the
parked pose and every finger/table closest point."""
import json
from pathlib import Path
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

run = Path("runs/20261112_145353_eb3331f0")
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

_orig_plan = solver.grasp_planner.plan


def plan_probe(obs):
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
    print("PLAN-ENTRY qpos:", np.round(np.asarray(obs["qpos"], float), 3), flush=True)
    pl = solver.grasp_planner
    pl.obstacles = [p.getBodyUniqueId(i, physicsClientId=pl.client)
                    for i in range(p.getNumBodies(physicsClientId=pl.client))]
    pl.obstacles.remove(pl.robot)
    inv = {p.getBodyUniqueId(i, physicsClientId=pl.client):
           p.getBodyInfo(i, physicsClientId=pl.client)[0].decode()
           for i in range(p.getNumBodies(physicsClientId=pl.client))}
    # opening check for candidate 0
    cands = pl.sample_antipodal_grasps(points=obs["object_pc"], normals=obs["object_normals"])
    p1, p2, centre, quat, score = cands[0]
    width = np.linalg.norm(p2 - p1)
    cfg = pl.config
    opened = np.full(2, (width - cfg.closed_gap) / 2) + cfg.opening_clearance / 2
    q = pl._full(np.asarray(obs["qpos"], float), np.asarray(obs["qpos"], float)[pl.arm_indices], opened)
    pl._set(q)
    for body in pl.obstacles:
        for c in p.getClosestPoints(pl.robot, body, 0.01, physicsClientId=pl.client):
            if c[3] in pl.finger_ids or c[8] < cfg.collision_margin:
                st = p.getLinkState(pl.robot, c[3], computeForwardKinematics=True,
                                    physicsClientId=pl.client)
                print("PLANENTRY-CONTACT", inv.get(body, body), "link", c[3],
                      p.getJointInfo(pl.robot, c[3], physicsClientId=pl.client)[12].decode(),
                      "link_pos", np.round(st[4], 3), "oblink", c[4], "dist", round(c[8], 5), flush=True)
    pl._set(np.asarray(obs["qpos"], float))  # undo probe perturbation (plan() checks staleness)
    return _orig_plan(obs)


solver.grasp_planner.plan = plan_probe

steps = 0
prev_state = solver.state
while steps < 1500 and solver.state != 4:
    obs = env._get_obs()
    a = np.asarray(solver.get_action(env), dtype=float)
    if solver.state != prev_state:
        print(f"state {prev_state}->{solver.state} at step {steps}", flush=True)
        print("  action:", np.round(a, 3), flush=True)
        print("  qpos:  ", np.round(np.asarray(obs["qpos"], float), 3), flush=True)
        prev_state = solver.state
    if 240 <= steps <= 260 or (225 <= steps <= 245 and prev_state == 0):
        q = np.asarray(obs["qpos"], float)
        print(f"step {steps} st={prev_state} arm={np.round(q[6:13],2)} "
              f"cmd_arm={np.round(a[6:13],2)} torso={q[3]:.2f}", flush=True)
    env.step(a)
    steps += 1
print(f"reached state {solver.state} at step {steps}", flush=True)
# run one more plan cycle
for _ in range(200):
    obs = env._get_obs()
    env.step(np.asarray(solver.get_action(env), dtype=float))
    if solver.state in (2, 3, 4):
        print(f"state {solver.state} at step {steps}", flush=True)
        break

obs = env._get_obs()
parked = np.asarray(obs["qpos"], float)
np.set_printoptions(precision=3, suppress=True, linewidth=200)
print("parked:", parked)
op = np.asarray(obs["object_pos"], float)
print("object:", op, "dist:", np.linalg.norm(parked[:2] - np.array([op[0]-2.0, op[1]])) if False else np.linalg.norm(np.array([parked[0]-2.0, parked[1]]) - op[:2]))

pl = solver.grasp_planner
pl.obstacles = [p.getBodyUniqueId(i, physicsClientId=pl.client)
                for i in range(p.getNumBodies(physicsClientId=pl.client))]
pl.obstacles.remove(pl.robot)
inv = {p.getBodyUniqueId(i, physicsClientId=pl.client):
       p.getBodyInfo(i, physicsClientId=pl.client)[0].decode()
       for i in range(p.getNumBodies(physicsClientId=pl.client))}

# open fingers at nominal width like plan()'s opening check
cands = pl.sample_antipodal_grasps(points=obs["object_pc"], normals=obs["object_normals"])
p1, p2, centre, quat, score = cands[0]
width = np.linalg.norm(p2 - p1)
cfg = pl.config
nominal = np.full(2, (width - cfg.closed_gap) / 2)
opened = nominal + cfg.opening_clearance / 2
q = pl._full(parked, parked[pl.arm_indices], opened)
pl._set(q)
for body in pl.obstacles:
    for c in p.getClosestPoints(pl.robot, body, 0.01, physicsClientId=pl.client):
        if c[3] in pl.finger_ids or body == 2:
            st = p.getLinkState(pl.robot, c[3], computeForwardKinematics=True,
                                physicsClientId=pl.client)
            print("CONTACT", inv.get(body, body), "link", c[3],
                  p.getJointInfo(pl.robot, c[3], physicsClientId=pl.client)[12].decode(),
                  "link_pos", np.round(st[4], 3), "oblink", c[4], "dist", round(c[8], 5))
