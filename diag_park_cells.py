import contextlib
import io
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0
_buf = io.StringIO()
with contextlib.redirect_stdout(_buf):
    from pick_nav_reach_env import PickNavReachEnv
    from solver import Solver

env = PickNavReachEnv(seed=42, object_idx=7, gui=False)
env.reset()
for _ in range(250):
    obs = env._get_obs()
    env.step(np.asarray(obs["qpos"], float))
solver = Solver(env)
for _ in range(260):
    solver.get_action(env)
obs = env._get_obs()
op = np.asarray(obs["object_pos"], float)
base = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])
print("base", base.round(2), "object", op[:2].round(2))

occ = solver.inflated_occupancy
cells = np.argwhere(occ == 0)
world = np.column_stack([
    solver.x_min + (cells[:, 1] + .5) * solver.cell_size,
    solver.y_max - (cells[:, 0] + .5) * solver.cell_size])
d_obj = np.linalg.norm(world - op[:2], axis=1)
d_rob = np.linalg.norm(world - base, axis=1)
ann = (d_obj >= 0.25) & (d_obj <= 0.90)
sel = np.nonzero(ann)[0]
sel.sort(key=lambda i: d_rob[i])
print("free annulus:", len(sel))
print("nearest-to-robot annulus cells (d_obj, d_rob):")
for i in sel[:6]:
    print("  ", world[i].round(2), f"d_obj={d_obj[i]:.2f} d_rob={d_rob[i]:.2f}")
# where would NavigateToGrasp send it (its own logic)?
g = solver.navigate_grasp
goal = g.nearest_free_goal(op[:2], occ, solver.convert_to_grid,
                           solver.convert_to_world, solver.cell_size,
                           robot_xy=base)
print("nearest_free_goal ->", goal, "d_obj",
      float(np.linalg.norm(np.array(goal) - op[:2])))
