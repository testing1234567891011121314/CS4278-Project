"""Trace one pick: every few ticks during the grasp and the back-off, print the
executor's phase, how far the object has risen, its tilt, the force on each
finger and the base speed.

    pnr/bin/python tools/experiments/trace_grasp.py OBJECT [EVERY=5] [TICKS=900] [CLOUD_SEED=300]
"""
import sys
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

from solver import Config, Solver, State

obj_id = int(sys.argv[1])
every = int(sys.argv[2]) if len(sys.argv) > 2 else 5
ticks = int(sys.argv[3]) if len(sys.argv) > 3 else 900
cloud = int(sys.argv[4]) if len(sys.argv) > 4 else 300
env = make_env(obj_id, 42, cloud)
solver = Solver(env, Config(seed=cloud))
client = env.pb_physics_client
rest, start_up, last_base, done, hand_start = None, None, None, 0, None
for tick in range(ticks):
    with quiet():
        obs, _ = env.step(solver.get_action())
    if solver.state in (State.GRASP, State.BACK_OFF) and solver.attempt.executor is not None:
        ex, robot = solver.attempt.executor, solver.scene.robot
        if rest is None or ex.phase in ("transit", "approach"):
            rest = float(obs["object_pos"][2])
            start_up = np.array(p.getMatrixFromQuaternion(obs["object_xyzw"])).reshape(3, 3)
        if tick % every == 0 and ex.phase not in ("transit",):
            force = {link: 0.0 for link in robot.finger_links}
            for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client):
                if c[3] in force:
                    force[c[3]] += c[9]
            turn = np.array(p.getMatrixFromQuaternion(obs["object_xyzw"])).reshape(3, 3)
            cos = (np.trace(turn @ start_up.T) - 1) / 2
            hand = np.array(p.getMatrixFromQuaternion(p.getLinkState(env.robot_id, robot.gripper_link,
                            computeForwardKinematics=True, physicsClientId=client)[5])).reshape(3, 3)
            if ex.phase in ("close", "approach") or hand_start is None:
                hand_start = hand
            hand_turn = np.degrees(np.arccos(np.clip((np.trace(hand @ hand_start.T) - 1) / 2, -1, 1)))
            base = np.asarray(obs["qpos"], dtype=float)[:2]
            speed = 0.0 if last_base is None else np.linalg.norm(base - last_base) / every
            print(f"tick {tick:4d} {solver.state.value:9s} {ex.phase:8s} risen {100 * (obs['object_pos'][2] - rest):6.1f} cm  "
                  f"turned {np.degrees(np.arccos(np.clip(cos, -1, 1))):5.1f} deg  hand {hand_turn:5.1f} deg  finger forces "
                  f"{force[robot.finger_links[0]]:6.1f} {force[robot.finger_links[1]]:6.1f} N  "
                  f"fingers {obs['qpos'][13]:.4f} {obs['qpos'][14]:.4f}  base {1000 * speed:5.1f} mm/tick")
            last_base = base
    if solver.state in (State.NAVIGATE, State.STOPPED):
        done += 1
        if done > 10:
            break
print("transitions:", solver.report.transitions[:14])
a = solver.attempt
print("last attempt:", a.failure, a.plan_stats)
print("earlier:", [(x["failure"], (x.get("plan_stats") or {}).get("grasp")) for x in solver.report.attempts][:4])
solver.close()
close_env(env)
