"""Can an object be picked once it overhangs the table edge? The experiment
(not the solver) moves the object so that it overhangs the edge nearest the
robot, then lets the solver pick it and measures the result.

    pnr/bin/python tools/experiments/overhang.py [OBJECT=11] [OVERHANG_M=0.10]
"""
import sys
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

from solver import Config, Solver, State

object_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 11
overhang = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
env = make_env(object_idx, 42, 300)
client = env.pb_physics_client
with quiet():
    for _ in range(110):
        obs, _ = env.step(obs["qpos"] if "obs" in dir() else env._get_obs()["qpos"])
lower, upper = p.getAABB(env.object_id, physicsClientId=client)
table = [b for b in range(p.getNumBodies(physicsClientId=client)) if p.getBodyInfo(b, physicsClientId=client)[1] == b"table.urdf"][0]
edge = p.getAABB(table, physicsClientId=client)[0][0]           # the table's edge toward the robot (-x)
position, orientation = p.getBasePositionAndOrientation(env.object_id, physicsClientId=client)
shift = (edge - overhang) - lower[0]
p.resetBasePositionAndOrientation(env.object_id, [position[0] + shift, position[1], position[2] + 0.002], orientation,
                                  physicsClientId=client)
with quiet():
    for _ in range(40):
        obs, _ = env.step(obs["qpos"])
rest = np.asarray(obs["object_pos"], dtype=float)
print(f"object rests at {np.round(rest, 3)}, overhanging the edge at x={edge:.3f} by {overhang} m; still on the table: {rest[2] > 0.3}")
solver = Solver(env, Config(seed=300))
fingers = [j for j in range(p.getNumJoints(env.robot_id, physicsClientId=client))
           if b"finger" in p.getJointInfo(env.robot_id, j, physicsClientId=client)[12]]
lifted, held_run, done = None, 0, 0
for tick in range(700):
    with quiet():
        obs, _ = env.step(solver.get_action())
    forces = {f: 0.0 for f in fingers}
    for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client):
        if c[3] in forces:
            forces[c[3]] += c[9]
    held = all(v > 0.5 for v in forces.values())
    risen = float(obs["object_pos"][2]) - rest[2]
    if lifted is None and held and risen >= 0.10:
        lifted = tick
    if lifted is not None:
        held_run = held_run + 1 if held else 0
    if solver.state in (State.NAVIGATE, State.STOPPED):
        done += 1
        if done > 15:
            break
print(f"lifted at tick {lifted}; held now {held} for {held_run} ticks; risen {100 * risen:.1f} cm; forces {[round(v) for v in forces.values()]} N; state {solver.state.value}")
tries = solver.report.attempts + [dict(failure=solver.attempt.failure, plan_stats=solver.attempt.plan_stats, search=solver.attempt.search)]
for a in tries[:4]:
    print("attempt:", a.get("failure"), "|", {k: v for k, v in (a.get("search") or {}).items() if k != "parking"}, "|", (a.get("plan_stats") or {}).get("grasp"))
solver.close()
close_env(env)
