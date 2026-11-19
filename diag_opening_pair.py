"""Replay the trial loop; at plan time dump the parked pose and the exact
obstacle the opening check collides with (body/link/world position)."""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

import json
import hashlib
from pathlib import Path
from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

spec = json.loads(Path("runs/20261112_145353_eb3331f0/maze_spec.json").read_text()) \
    if Path("runs/20261112_145353_eb3331f0/maze_spec.json").exists() else None
print("spec found:", spec is not None)

env = PickNavReachEnv(seed=42, object_idx=7, gui=False)
env.reset()
solver = Solver(env)

steps = 0
while steps < 1500 and solver.state not in (1, 4):
    obs = env._get_obs()
    env.step(np.asarray(solver.get_action(env), dtype=float))
    steps += 1
print(f"reached state {solver.state} at step {steps}")
if solver.state != 1:
    raise SystemExit

for _ in range(14):  # settle ticks
    obs = env._get_obs()
    env.step(np.asarray(solver.get_action(env), dtype=float))
obs = env._get_obs()
parked = np.asarray(obs["qpos"], float)
print("parked qpos:", np.round(parked, 3))

pl = solver.grasp_planner
cands = pl.sample_antipodal_grasps(points=obs["object_pc"], normals=obs["object_normals"])
p1, p2, centre, quat, score = cands[0]
width = np.linalg.norm(p2 - p1)
cfg = pl.config
nominal = np.full(2, (width - cfg.closed_gap) / 2)
opened = nominal + cfg.opening_clearance / 2
q = pl._full(parked, parked[pl.arm_indices], opened)
pl._set(q)
inv = {}
for b in [p.getBodyUniqueId(i, physicsClientId=pl.client)
          for i in range(p.getNumBodies(physicsClientId=pl.client))]:
    inv[b] = p.getBodyInfo(b, physicsClientId=pl.client)[0].decode()
for c in p.getClosestPoints(pl.robot, pl.robot, cfg.collision_margin,
                            physicsClientId=pl.client):
    if frozenset((c[3], c[4])) in pl.self_pairs:
        print("SELFPAIR", inv.get(-1, "?"), c[3], c[4], "dist", round(c[8], 5))
pl.obstacles = [p.getBodyUniqueId(i, physicsClientId=pl.client)
                for i in range(p.getNumBodies(physicsClientId=pl.client))]
pl.obstacles.remove(pl.robot)
for body in pl.obstacles:
    for c in p.getClosestPoints(pl.robot, body, cfg.collision_margin,
                                physicsClientId=pl.client):
        if c[8] < cfg.collision_margin:
            link = c[3]
            st = p.getLinkState(pl.robot, link, computeForwardKinematics=True,
                                physicsClientId=pl.client)
            print("OBSTACLE", inv.get(body, body), "link", link,
                  p.getJointInfo(pl.robot, link, physicsClientId=pl.client)[12].decode(),
                  "link_pos", np.round(st[4], 3),
                  "obstacle_link", c[4], "dist", round(c[8], 5))
