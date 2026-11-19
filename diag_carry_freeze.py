"""Replay r001's logged targets (no solver, no plan budget -> deterministic)
and instrument the freeze window: every contact involving the robot/object
with force, object-vs-world in particular (the jam dump excluded the object)."""
import json
import re
from pathlib import Path
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv

run = Path("runs/20261113_212841_c428c17c")
spec = json.loads((run / "maze.json").read_text(encoding="utf-8"))
maze_path = Path(spec["urdf_path"])
log = (run / "object_07_013_apple_repeat_001" / "worker.log").read_text(
    encoding="utf-8", errors="replace")

pat = re.compile(
    r"=====+Step (\d+)=====+\s*\n"
    r"current qpos:\s*\[([^\]]+)\]\s*\n"
    r"target:\s*\[([^\]]+)\]", re.S)
steps = {}
for m in pat.finditer(log):
    n = int(m.group(1))
    q = np.array([float(v) for v in m.group(2).split()])
    t = np.array([float(v) for v in m.group(3).split()])
    if q.size == 15 and t.size == 15:
        steps[n] = (q, t)
print("parsed steps:", len(steps), "range", min(steps), max(steps))


class FixedMazeEnv(PickNavReachEnv):
    def _load_maze(self):
        self.maze_out_pos_x, self.maze_out_pos_y = spec["goal_xy"]
        self.cube_positions = np.asarray(spec["cube_positions"], dtype=float)
        p.loadURDF(str(maze_path), useFixedBase=True,
                   flags=p.URDF_MERGE_FIXED_LINKS,
                   physicsClientId=self.pb_physics_client)


env = FixedMazeEnv(seed=42, object_idx=7, gui=False)
obs = env.reset()
q0 = np.asarray(obs["qpos"], float)
print("qpos diff vs log step0:",
      np.max(np.abs(q0 - steps[min(steps)][0])))

client = env.pb_physics_client
obj_id = env.object_id
for _ in range(5):
    env.step(np.asarray(obs["qpos"], float))
for c in p.getContactPoints(bodyA=env.robot_id,
                            physicsClientId=client)[:2]:
    print("RAWCT", [(i, v if not hasattr(v, "__len__") else "SEQ")
                    for i, v in enumerate(c)])
WIN = (1340, 1745)

names = {}
for i in range(p.getNumBodies(physicsClientId=client)):
    names[i] = p.getBodyInfo(i, physicsClientId=client)[0].decode()
print("bodies:", names)


def dump(n, qpos, target, obs):
    obj = np.asarray(obs["object_pos"], float)
    base = np.array([qpos[0] - 2.0, qpos[1]])
    lines = [f"--- step {n} base=({base[0]:.3f},{base[1]:.3f}) "
             f"d(target-q)=({target[0]-qpos[0]:+.4f},{target[1]-qpos[1]:+.4f}) "
             f"qvelxy={np.round(obs['qvel'][:2], 3)} "
             f"obj=({obj[0]:.3f},{obj[1]:.3f},{obj[2]:.3f})"]
    for bA, tag in ((obj_id, "OBJ"), (env.robot_id, "ROB")):
        for bB, nm in names.items():
            if bB == bA:
                continue
            cs = [c for c in p.getContactPoints(
                bodyA=bA, bodyB=bB, physicsClientId=client) if c[9] > 0.2]
            if not cs:
                continue
            top = sorted(cs, key=lambda c: -c[9])[:4]
            for c in top:
                rl = c[3] if bA == env.robot_id else c[4]
                lk = (p.getJointInfo(env.robot_id, rl,
                                     physicsClientId=client)[12].decode()
                      if rl >= 0 else "base")
                lines.append(
                    f"    {tag}x{nm} lkA={c[3]} lkB={c[4]} {lk} "
                    f"F={c[9]:.1f} n={len(cs)} "
                    f"pt={np.round(c[6], 2)} nrm={np.round(c[8], 2)}")
    print("\n".join(lines), flush=True)


for n in sorted(steps):
    q, t = steps[n]
    obs, _ = env.step(t)
    if not (WIN[0] <= n <= WIN[1]):
        continue
    if n % 20 == 0 or n in (1351, 1352):
        dump(n, q, t, obs)
    if n == 1500:
        print("=== FROZEN-TICK PHYSICS PROBE ===")
        for j in (0, 1, 2):
            st = p.getJointState(env.robot_id, j, physicsClientId=client)
            print(f"joint {j} q={np.round(st[0],4)} qd={np.round(st[1],4)} "
                  f"tf={np.round(st[2],1)} raw_info_len="
                  f"{len(p.getJointInfo(env.robot_id, j, physicsClientId=client))}")
        for b in range(p.getNumBodies(physicsClientId=client)):
            if b == env.robot_id:
                continue
            cs = p.getContactPoints(bodyA=env.robot_id, bodyB=b,
                                    physicsClientId=client)
            if cs:
                top = max(c[9] for c in cs)
                lk = sorted({c[3] for c in cs if c[9] > 0.01})
                print(f"robot x {names[b]}: n={len(cs)} maxF={top:.1f} "
                      f"links={lk}")
        # what does the plane feel under the wheels?
        cs = p.getContactPoints(bodyB=env.robot_id, physicsClientId=client)
        print("robot total contacts:", len(cs),
              "maxF:", max((c[9] for c in cs), default=0.0))

q = np.asarray(env._get_obs()["qpos"], float)
print("final base:", np.array([q[0] - 2.0, q[1]]), "yaw:", q[2])
