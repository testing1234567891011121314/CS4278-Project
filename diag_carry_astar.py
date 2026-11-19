"""Replay to carry start (FixedMazeEnv); build the carry map and inspect
connectivity between the base cell and the stand cell near the goal."""
import json
from pathlib import Path
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver
from custom_libraries.a_star import a_star

run = Path("runs/20261112_223559_84cad31e")
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

steps = 0
while steps < 1500 and solver.state != 3:
    obs = env._get_obs()
    env.step(np.asarray(solver.get_action(env), dtype=float))
    steps += 1
print(f"state {solver.state} at step {steps}", flush=True)

# Place the base at the escape cell the real run planned from.
import pybullet as p2
for idx, val in zip(env.joint_indices[:2], [1.0376, 0.25]):
    p.resetJointState(env.robot_id, idx, float(val))
obs = env._get_obs()
q = np.asarray(obs["qpos"], float)
base_xy = np.array([q[0] - 2.0, q[1]])
goal = np.asarray(obs["goal_pos"], float)
print("base:", np.round(base_xy, 3))

cmap = solver.carry_occupancy
nmap = solver.inflated_occupancy
start_cell = tuple(np.asarray(solver.convert_to_grid(base_xy), dtype=int))
if cmap[start_cell] != 0:
    free = np.argwhere(cmap == 0)
    start_cell = tuple(free[np.argmin(
        np.linalg.norm(free - np.asarray(start_cell), axis=1))])
dirv = base_xy - goal[:2]
d = float(np.linalg.norm(dirv))
stand = goal[:2] + 0.90 * dirv / d
stand_cell = tuple(np.asarray(solver.convert_to_grid(stand), dtype=int))
if cmap[stand_cell] != 0:
    free = np.argwhere(cmap == 0)
    stand_cell = tuple(free[np.argmin(
        np.linalg.norm(free - np.asarray(stand_cell), axis=1))])
print("start cell", start_cell, "free:", cmap[start_cell] == 0)
print("stand cell", stand_cell, "free:", cmap[stand_cell] == 0)

path = a_star(start_cell, stand_cell, cmap)
print("carry a_star len:", len(path))
path2 = a_star(start_cell, stand_cell, nmap)
print("nav-map a_star len:", len(path2))

# BFS reachability from start on carry map
from collections import deque
rows, cols = cmap.shape
seen = np.zeros_like(cmap, dtype=bool)
if cmap[start_cell] == 0:
    dq = deque([start_cell])
    seen[start_cell] = True
    while dq:
        r, c = dq.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and not seen[nr, nc] \
                        and cmap[nr, nc] == 0:
                    seen[nr, nc] = True
                    dq.append((nr, nc))
print("reachable cells:", int(seen.sum()), "of free:", int((cmap == 0).sum()))
print("stand reachable:", bool(seen[stand_cell]))

# downsample map ascii (every 4 cells), rows y-desc
print("\ncarry map (S=start, G=stand, #=blocked, .=free):")
R, C = rows // 4, cols // 4
g = np.zeros((R, C), dtype="<U1")
for r in range(R):
    for c in range(C):
        blk = cmap[4*r:4*r+4, 4*c:4*c+4]
        g[r, c] = "#" if blk.any() else "."
sr, sc = start_cell[0] // 4, start_cell[1] // 4
gr_, gc_ = stand_cell[0] // 4, stand_cell[1] // 4
g[sr, sc] = "S"
g[gr_, gc_] = "G"
for r in range(R):
    print("".join(g[r]))
