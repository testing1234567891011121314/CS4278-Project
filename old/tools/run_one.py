"""Headless single-trial runner for the Pick-Nav-Reach project.

Usage: python run_one.py PROJECT_DIR --object IDX --seed S --max-steps N [--log FILE]
Writes a JSON summary line to stdout at the end (prefixed with RESULT:).
"""
import argparse
import json
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("project")
ap.add_argument("--object", type=int, default=5)
ap.add_argument("--seed", type=int, default=42)
ap.add_argument("--max-steps", type=int, default=4000)
ap.add_argument("--maze-copy", default=None, help="private maze urdf path (for parallel runs)")
args = ap.parse_args()

project = os.path.abspath(args.project)
os.chdir(project)
sys.path.insert(0, project)
time.sleep = lambda *_a, **_k: None  # env.step sleeps 1/240 per substep; skip in headless runs

import numpy as np
import pybullet as p
from pick_nav_reach_env_multi import PickNavReachEnv
from solver import Solver

maze = args.maze_copy or f"/tmp/maze_{os.getpid()}.urdf"
env = PickNavReachEnv(seed=args.seed, object_idx=args.object, maze_urdf_path=maze, gui=False)
env.reset()
solver = Solver(env)

table_z = None
init_obj_z = None
max_obj_z = -1.0
states = []
result = dict(object=args.object, seed=args.seed)
t0 = time.perf_counter()
for step in range(args.max_steps):
    action = solver.get_action(env)
    if action is None:
        result["stopped"] = f"solver returned None at step {step} (state {solver.state})"
        break
    obs, info = env.step(action)
    z = float(obs["object_pos"][2])
    if step == 250:
        init_obj_z = z
    if step > 250:
        max_obj_z = max(max_obj_z, z)
    if not states or states[-1][1] != solver.state:
        states.append((step, solver.state))
    if solver.state == 3 and getattr(solver, "_hold_started", None) is None:
        solver._hold_started = step
    goal_d = float(np.linalg.norm(np.asarray(obs["object_pos"]) - np.asarray(obs["goal_pos"])))
    result["min_goal_dist"] = min(result.get("min_goal_dist", 1e9), goal_d)
    result["final_goal_dist"] = goal_d
    if solver.state in (3, 5, 67) and step - solver._hold_started > 100:
        result["held_100"] = True
    if goal_d < 0.10:
        result["stopped"] = "REACHED goal"
        break
    if solver.state == 4:
        result["stopped"] = "failed state 4"
        break

result.update(
    steps=step + 1,
    wall_s=round(time.perf_counter() - t0, 1),
    states=states,
    final_state=solver.state,
    grasp_error=getattr(solver, "grasp_error", None),
    init_obj_z=init_obj_z,
    max_obj_z=max_obj_z,
    lift_cm=None if init_obj_z is None else round(100 * (max_obj_z - init_obj_z), 1),
    planner_stats=getattr(solver.grasp_planner, "last_stats", None),
)
print("RESULT:" + json.dumps(result, default=str), flush=True)
