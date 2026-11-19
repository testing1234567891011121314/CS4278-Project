"""Which cells can the grasp-park annulus actually use?

Compares inflated_occupancy (wall-projection incl. arm sweep) vs a
base-radius-only grid for cells in the 0.25-0.90 m annulus around the
object.
"""
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

# settle solver state to 0 past the 200-wait
for _ in range(260):
    solver.get_action(env)

obs = env._get_obs()
op = np.asarray(obs["object_pos"], float)
print("object", np.round(op, 3))

cells = np.argwhere(solver.occupancy == 0)
d = np.linalg.norm(
    np.column_stack([solver.x_min + (cells[:, 1] + .5) * solver.cell_size,
                     solver.y_max - (cells[:, 0] + .5) * solver.cell_size])
    - op[:2], axis=1)
ann = (d >= 0.25) & (d <= 0.90)
ann_cells = cells[ann]
ann_world = np.column_stack([
    solver.x_min + (ann_cells[:, 1] + .5) * solver.cell_size,
    solver.y_max - (ann_cells[:, 0] + .5) * solver.cell_size])
infl = solver.inflated_occupancy
free_infl = ann_cells[infl[ann_cells[:, 0], ann_cells[:, 1]] == 0]
print(f"annulus cells: {len(ann_cells)}  free under ARM-projection grid: {len(free_infl)}")
if len(ann_cells):
    print(" annulus dist range", d[ann].min().round(2), d[ann].max().round(2))
if len(free_infl):
    df = np.linalg.norm(ann_world[infl[ann_cells[:,0], ann_cells[:,1]]==0] - op[:2], axis=1)
    print(" free-armgrid dist:", df.min().round(2), "median", np.median(df).round(2),
          df.max().round(2))
else:
    print(" ARM grid: annulus fully blocked -- nearest allowed:", )
    free_all = cells[infl[cells[:,0], cells[:,1]] == 0]
    w = np.column_stack([solver.x_min + (free_all[:,1]+.5)*solver.cell_size,
                         solver.y_max - (free_all[:,0]+.5)*solver.cell_size])
    dd = np.linalg.norm(w - op[:2], axis=1)
    print("   nearest free cell", dd.min().round(2), "m from object")

# base-only grid: walls inflated by base radius only (no arm projection)
from custom_libraries.a_star import a_star  # noqa
r_cells = int(np.ceil((solver.robot_radius + 0.05) / solver.cell_size))
walls = solver.wall_occupancy
rows, cols = walls.shape
padded = np.pad(walls.astype(bool), r_cells, mode="constant")
base_blocked = np.zeros((rows, cols), dtype=bool)
for dr in range(2 * r_cells + 1):
    for dc in range(2 * r_cells + 1):
        base_blocked |= padded[dr:dr + rows, dc:dc + cols]
base_grid = (base_blocked | solver.extra_occupancy.astype(bool)).astype(np.uint8)
free_base = ann_cells[base_grid[ann_cells[:, 0], ann_cells[:, 1]] == 0]
print(f"free under BASE-only grid: {len(free_base)}", )
if len(free_base):
    wb = np.column_stack([solver.x_min + (free_base[:,1]+.5)*solver.cell_size,
                          solver.y_max - (free_base[:,0]+.5)*solver.cell_size])
    db = np.linalg.norm(wb - op[:2], axis=1)
    print(" base-grid free dist:", db.min().round(2), "median", np.median(db).round(2),
          db.max().round(2))
