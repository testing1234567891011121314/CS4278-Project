"""Why does state 0 never hand off? Log goal vs base vs object each step."""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

env = PickNavReachEnv(seed=42, object_idx=7, gui=False)
env.reset()
solver = Solver(env)

for step in range(600):
    obs = env._get_obs()
    action = solver.get_action(env)
    if solver.state == 1:
        print(f"step {step}: HANDOFF to state 1")
        break
    if solver.state == 4:
        print(f"step {step}: state 4 (failed)")
        break
    base = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])
    obj = np.asarray(obs["object_pos"], float)[:2]
    d_obj = np.linalg.norm(base - obj)
    bearing = np.arctan2(obj[1] - obs["qpos"][1], obj[0] - obs["qpos"][0])
    err = (bearing - obs["qpos"][2] + np.pi) % (2 * np.pi) - np.pi
    if step % 25 == 0 or solver.state != 0:
        decision = solver.navigate_grasp.update(
            obs, solver.inflated_occupancy, solver.convert_to_grid,
            solver.convert_to_world, solver.cell_size)
        g = decision["navigation_goal"]
        gerr = "None" if g is None else f"{np.linalg.norm(base - np.asarray(g)):.3f}"
        gpb = solver.navigate_grasp.relative_grasp_pose(obs)[0] if solver.navigate_grasp.grasp else None
        cls = solver.navigate_grasp.classify_reachability(gpb) if gpb is not None else "no-grasp"
        d3 = None if gpb is None else round(float(np.linalg.norm(gpb)), 3)
        print(f"step {step:4d} state={solver.state} base=({base[0]:.2f},{base[1]:.2f}) "
              f"d_obj={d_obj:.2f} bear_err={err:.2f} dec={decision['state']} "
              f"d_goal={gerr} d3={d3} cls={cls}")
    env.step(np.asarray(action, dtype=float))
print("final state:", solver.state)
