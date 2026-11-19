"""Replay the batch (FixedMazeEnv + stored maze); at the carry jam dump base
pose, blocked cells nearby, and what body the base is touching."""
import json
from pathlib import Path
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

run = Path("runs/20261113_212841_c428c17c")
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

np.set_printoptions(precision=3, suppress=True, linewidth=200)

steps = 0
frozen = 0
while steps < 4200:
    obs = env._get_obs()
    a = np.asarray(solver.get_action(env), dtype=float)
    env.step(a)
    steps += 1
    if getattr(solver, "_carry_stage", None) == "travel":
        q = np.asarray(obs["qpos"], float)
        base = np.array([q[0] - 2.0, q[1]])
        cmd = getattr(solver, "_carry_cmd_xy", None)
        speed = float(np.linalg.norm(np.asarray(obs["qvel"], float)[:2]))
        if speed < 0.005 and cmd is not None:
            frozen += 1
        else:
            frozen = 0
        if steps % 50 == 0 or frozen in (1, 10, 40, 80, 150):
            print(f"step {steps} base=({base[0]:.3f},{base[1]:.3f}) "
                  f"cmd={None if cmd is None else np.round(cmd, 3)} "
                  f"action_xy=({a[0]-2.0:.3f},{a[1]:.3f}) speed={speed:.4f} "
                  f"frozen={frozen} stuck={getattr(solver,'_carry_stuck',0)} "
                  f"replans={getattr(solver,'_carry_replans',0)} "
                  f"servo={getattr(solver,'_carry_servo',False)}", flush=True)
        if frozen >= 200:
            break

q = np.asarray(env._get_obs()["qpos"], float)
base_xy = np.array([q[0] - 2.0, q[1]])
print("\nfinal base:", base_xy, "yaw:", q[2])
print("object:", np.asarray(env._get_obs()["object_pos"], float))

cell = solver.convert_to_grid(base_xy)
print("base cell:", cell)
r0, c0 = cell
sl = solver.carry_occupancy[max(0,r0-6):r0+7, max(0,c0-8):c0+9]
print("carry occupancy slice (rows=y desc, 1=blocked):")
print(sl.astype(int))

client = env.pb_physics_client
for i in range(p.getNumBodies(physicsClientId=client)):
    b = p.getBodyUniqueId(i, physicsClientId=client)
    nm = p.getBodyInfo(b, physicsClientId=client)[0].decode()
    for c in p.getClosestPoints(env.robot_id, b, 0.05, physicsClientId=client):
        if c[8] < 0.05 and b != env.object_id:
            st = p.getLinkState(env.robot_id, c[3], computeForwardKinematics=True,
                                physicsClientId=client)
            print("NEAR", nm, "link",
                  p.getJointInfo(env.robot_id, c[3], physicsClientId=client)[12].decode(),
                  "pos", np.round(st[4], 3), "oblink", c[4], "dist", round(c[8], 4))

for j in (0, 1, 4, 5):
    st = p.getJointState(env.robot_id, j, physicsClientId=client)
    print("root/head joint", j, "pos", np.round(st[0], 4), "vel", np.round(st[1], 4))
base_state = p.getLinkState(env.robot_id, 2, computeForwardKinematics=True,
                            physicsClientId=client)
print("chassis pos", np.round(base_state[4], 3))
