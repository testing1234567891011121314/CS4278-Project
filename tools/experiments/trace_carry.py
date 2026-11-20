"""Trace the carry of one object: every few ticks, the carry's phase, how far
along the route the object is, its speed, how far the object has slipped in
the hand since the carry began (along the fingers, across the pads, and its
turn about each of the hand's axes), what else the object touches, and the force on each finger.

    pnr/bin/python tools/experiments/trace_carry.py OBJECT [EVERY=20] [CLOUD_SEED=300] [FROM_TICK] [NAME=VALUE ...]

NAME=VALUE pairs override Config fields (give FROM_TICK first, e.g. 99999).

From FROM_TICK on, every tick is printed, with the base's turn and the
wrist's roll commanded in that tick.
"""
import sys
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

from solver import Config, Solver, State

object_idx = int(sys.argv[1])
every = int(sys.argv[2]) if len(sys.argv) > 2 else 20
cloud = int(sys.argv[3]) if len(sys.argv) > 3 else 300
closely = int(sys.argv[4]) if len(sys.argv) > 4 else 10 ** 9
import ast
overrides = {name: ast.literal_eval(value) for name, value in (pair.split("=", 1) for pair in sys.argv[5:])}
last_command = None
env = make_env(object_idx, 42, cloud)
solver = Solver(env, Config(seed=cloud, **overrides))
client = env.pb_physics_client


def in_hand(obs):
    """Object position and rotation in the gripper frame."""
    robot = solver.scene.robot
    state = p.getLinkState(env.robot_id, robot.gripper_link, computeForwardKinematics=True, physicsClientId=client)
    turn = np.array(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
    obj = np.array(p.getMatrixFromQuaternion(obs["object_xyzw"])).reshape(3, 3)
    return turn.T @ (np.asarray(obs["object_pos"], dtype=float) - np.array(state[4])), turn.T @ obj


start, last_along, shown = None, None, 0
for tick in range(3000):
    with quiet():
        obs, info = env.step(solver.get_action())
    follower = solver.attempt.carry
    if solver.state in (State.NAVIGATE, State.REACH) and follower is not None:
        position, turn = in_hand(obs)
        if start is None:
            start = (position, turn, tick)
            print("carry plan:", solver.attempt.plan_stats.get("carry"), "grasp:", solver.attempt.plan_stats.get("grasp"))
        command = (follower._yaw, follower._target[solver.scene.robot.arm[-1]], obs["qpos"][solver.scene.robot.arm[-1]])
        if tick >= closely and last_command is not None:
            print(f"   tick {tick}: base turned {np.degrees(command[0] - last_command[0]):+.2f} deg, roll target "
                  f"{np.degrees(command[1] - last_command[1]):+.2f} deg to {np.degrees(command[1]):+.1f}, roll measured "
                  f"{np.degrees(command[2]):+.1f}, base at {np.round(solver.scene.robot.base_pose(obs['qpos']), 3)}")
        last_command = command
        if (tick - start[2]) % every == 0 or tick >= closely:
            slip = 1000 * (position - start[0])
            cos = (np.trace(turn @ start[1].T) - 1) / 2
            robot = solver.scene.robot
            force = {link: 0.0 for link in robot.finger_links}
            for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client):
                if c[3] in force:
                    force[c[3]] += c[9]
            # The turn since the carry began, about the hand's own axes:
            # along the fingers, along the pads' line, across both.
            from custom_libraries.grasping.ik import rotation_vector
            axes = "(fingers {:+.0f}, pads' line {:+.0f}, across {:+.0f})".format(*np.degrees(rotation_vector(turn @ start[1].T)))
            others = {(c[2], c[4]) for c in p.getContactPoints(bodyA=env.object_id, physicsClientId=client)
                      if not (c[2] == env.robot_id and c[4] in robot.finger_links)}
            walls = {(c[2], c[3]) for c in p.getContactPoints(bodyA=env.robot_id, physicsClientId=client)
                     if c[2] not in (env.robot_id, env.object_id) and c[3] in solver.scene.collision.arm_links}
            touching = (f"  object touches (body, link) {sorted(others)}" if others else "") + \
                (f"  arm link touches (body, arm link) {sorted(walls)}" if walls else "") + \
                f"  fingers at {np.round(1000 * np.asarray(obs['qpos'])[list(robot.fingers)], 1)} mm"
            speed = 0.0 if last_along is None else 1000 * (follower._along - last_along) / every
            last_along = follower._along
            print(f"tick {tick:4d} {solver.state.value:8s} {follower.phase:11s} along {follower._along:5.2f} of {follower.plan.length:5.2f} m  "
                  f"{speed:4.1f} mm/tick  slip along fingers {slip[0]:6.1f} across {slip[1]:6.1f} side {slip[2]:6.1f} mm  "
                  f"turned {np.degrees(np.arccos(np.clip(cos, -1, 1))):5.1f} deg {axes}  forces {force[robot.finger_links[0]]:5.0f} {force[robot.finger_links[1]]:5.0f} N{touching}")
    if start is not None and solver.state not in (State.NAVIGATE, State.REACH):
        break
print("ended:", solver.state.value, "tick", solver.tick, "failure:", solver.attempt.failure, "| earlier:",
      [a["failure"] for a in solver.report.attempts][:3], f"| {100 * float(info['dist_to_goal']):.0f} cm from the goal")
solver.close()
close_env(env)
