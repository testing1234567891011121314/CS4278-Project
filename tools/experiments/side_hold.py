"""Side hold for the carry: after the back-off, turn the hand about the line
between the pads until the object's centre of mass is straight below (or
above) the grip, then let the solver carry it as usual.

The object can turn almost freely about the pads' line (the env has no
spinning friction), so a centre of mass beside that line is pulled round by
gravity all through the carry. Below or above the line, gravity has no lever.

    pnr/bin/python tools/experiments/side_hold.py OBJECT [WAY=below|above|roll|none|DEGREES] [CLOUD_SEED=300] [NAME=VALUE ...]

WAY "roll" is for a pads' line that is not level (a plate held flat by its
rim): the hand is rolled about its own axis until the line is level and the
centre of mass is below it.

NAME=VALUE pairs override Config fields.

WAY as a number turns the hand that many degrees about the pads' line whatever
the centre of mass does (a long object gripped at its middle has no lever, but
stood on end it no longer lags the base's turns).

Prints the geometry at the grasp, how far the turn got, whether the object
followed the hand, and then the carry every 100 ticks.
"""
import sys
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

from custom_libraries.common import Budget
from custom_libraries.grasping import ik
from custom_libraries.grasping.executor import PathStepper, is_held
from custom_libraries.planning_sim import rotation
from solver import Config, Solver, State

object_idx = int(sys.argv[1])
way = sys.argv[2] if len(sys.argv) > 2 else "below"
cloud = int(sys.argv[3]) if len(sys.argv) > 3 else 300
env = make_env(object_idx, 42, cloud)
import ast
overrides = {name: ast.literal_eval(value) for name, value in (pair.split("=", 1) for pair in sys.argv[4:])}
solver = Solver(env, Config(seed=cloud, **overrides))
client = env.pb_physics_client


def about(axis, angle):
    """Rotation by `angle` about the unit vector `axis`."""
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def hand():
    state = p.getLinkState(env.robot_id, solver.scene.robot.gripper_link, computeForwardKinematics=True,
                           physicsClientId=client)
    return np.array(state[4]), rotation(state[5])


def grip_point():
    """Where the pads hold the object: the force-weighted middle of the finger contacts."""
    points = [(np.array(c[6]), c[9]) for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client)
              if c[3] in solver.scene.robot.finger_links and c[9] > 0]
    weight = sum(f for _, f in points)
    return sum(x * f for x, f in points) / weight if weight > 0 else None


def lever(obs, grip, closing):
    """From the pads' line to the centre of mass, and its angle from straight down, degrees."""
    off = np.asarray(obs["object_pos"], dtype=float) - grip
    off = off - (off @ closing) * closing
    down = np.array([0.0, 0.0, -1.0]) - (-closing[2]) * closing
    down = down / np.linalg.norm(down)
    return off, float(np.degrees(np.arctan2(closing @ np.cross(down, off), down @ off)))


# ---------------------------------------------------------------- the pick
obs = env._get_obs()
while solver.tick < 1500 and not (solver.state is State.NAVIGATE and solver.attempt.carry is None):
    if solver.state in (State.DONE, State.STOPPED):
        break
    with quiet():
        obs, info = env.step(solver.get_action())
if solver.state is not State.NAVIGATE:
    print("no pick:", solver.state.value, [a["failure"] for a in solver.report.attempts], solver.attempt.failure)
    solver.close(), close_env(env)
    sys.exit()

scene, robot = solver.scene, solver.scene.robot
position, turn = hand()
closing, approach = turn[:, 1], turn[:, 0]
grip = grip_point()
off, angle = lever(obs, grip, closing)
print(f"tick {solver.tick}: picked, grasp {solver.attempt.plan_stats.get('grasp')}")
print(f"  hand points {np.round(approach, 2)}, pads' line {np.round(closing, 2)} "
      f"({np.degrees(np.arcsin(abs(closing[2]))):.0f} deg from level), grip {np.round(grip, 3)}")
print(f"  centre of mass {1000 * np.linalg.norm(off):.0f} mm from the pads' line, {angle:+.0f} deg from straight down")

# ---------------------------------------------------------------- the turn
taken = 0
if way != "none":
    if way in ("below", "above"):
        wanted = -angle if way == "below" else float(-angle + np.sign(angle or 1.0) * 180.0)
        wanted = (wanted + 180.0) % 360.0 - 180.0
    elif way == "roll":
        # Roll about the hand's own axis: of the two rolls that level the
        # pads' line, the one that leaves the centre of mass lower.
        tilt = np.arctan2(closing[2], turn[2, 2])            # closing' = cos(a) closing + sin(a) (approach x closing)
        full = np.asarray(obs["object_pos"], dtype=float) - grip
        choices = [(-tilt + k * np.pi + np.pi) % (2 * np.pi) - np.pi for k in (0, 1)]
        wanted = float(np.degrees(min(choices, key=lambda a: (about(approach, a) @ full)[2])))
        closing_axis, closing = closing, approach            # the turn below goes about `closing`
    else:
        wanted = float(way)
    scene.collision.hold(solver._hold, obs["object_pos"], obs["object_xyzw"])
    scene.collision.focus(robot.base_pose(solver._hold)[:2])
    q, path, stopped = solver._hold.copy(), [solver._hold.copy()], None
    joints = list(robot.plan_joints)
    for k in range(1, int(abs(wanted) / 2.0) + 2):
        part = np.radians(np.sign(wanted) * min(abs(wanted), 2.0 * k))
        spin = about(closing, part)
        nxt = ik.solve(scene.sim, grip + spin @ (position - grip), spin @ turn, q, solver.rng, Budget(5000),
                       restarts=0)
        if nxt is None:
            stopped = "out of reach"
        elif np.abs(nxt[joints] - q[joints]).max() > 0.3:
            stopped = "joint jump"
        elif not scene.collision.is_free(nxt, 0.01, load_margin=0.01):
            stopped = f"blocked {scene.collision.clearances(nxt, with_object=False)}"
        if stopped:
            print(f"  turn stopped at {np.degrees(part):+.0f} of {wanted:+.0f} deg: {stopped}")
            break
        q = nxt
        path.append(q)
    scene.collision.release()
    limit = np.full(len(q), 0.03)
    limit[robot.torso] = 0.004
    stepper = PathStepper(np.array(path), limit)
    start_turn = rotation(obs["object_xyzw"])
    action = solver._hold
    while not stepper.finished or taken % 20:
        if not stepper.finished:
            action = stepper.step()
        with quiet():
            obs, info = env.step(action)
        taken += 1
        if taken % 20 == 0:
            now_position, now_turn = hand()
            grip_now = grip_point()
            lever_now = lever(obs, grip_now, now_turn[:, 1])[1] if grip_now is not None else float("nan")
            print(f"  +{taken:3d} ticks: hand turned {np.degrees(np.linalg.norm(ik.rotation_vector(now_turn @ turn.T))):5.1f} deg, "
                  f"object {np.degrees(np.linalg.norm(ik.rotation_vector(rotation(obs['object_xyzw']) @ start_turn.T))):5.1f} deg, "
                  f"centre of mass {lever_now:+6.1f} deg from straight down, held {is_held(env, robot)}")
    for _ in range(40):
        with quiet():
            obs, info = env.step(action)
    taken += 40
    solver._hold = np.asarray(action, dtype=float).copy()
    solver.tick += taken
    grip_now = grip_point()
    if grip_now is not None:
        off, angle = lever(obs, grip_now, hand()[1][:, 1])
        print(f"  settled: hand points {np.round(hand()[1][:, 0], 2)}, centre of mass {1000 * np.linalg.norm(off):.0f} mm "
              f"from the pads' line, {angle:+.0f} deg from straight down, object z {obs['object_pos'][2]:.3f}")

# ---------------------------------------------------------------- the carry
began, start = solver.tick, None
while solver.tick < 3000 and solver.state not in (State.DONE, State.STOPPED):
    with quiet():
        obs, info = env.step(solver.get_action())
    if solver.state not in (State.NAVIGATE, State.REACH):
        break
    if start is None and solver.attempt.carry is not None:
        start = (hand()[1].T @ rotation(obs["object_xyzw"]),)
        grip_now = grip_point()
        off, angle = lever(obs, grip_now, hand()[1][:, 1])
        print(f"  tick {solver.tick}: hold {solver.attempt.plan_stats.get('hold')}: hand points {np.round(hand()[1][:, 0], 2)}, "
              f"pads' line {np.round(hand()[1][:, 1], 2)}, centre of mass {1000 * np.linalg.norm(off):.0f} mm from it, "
              f"{angle:+.0f} deg from straight down")
        print("  carry plan:", solver.attempt.plan_stats.get("carry"))
    if start is None:
        if solver.tick % 10 == 0:                       # taking the hold: forces, and what else the object touches
            force = [sum(c[9] for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client)
                         if c[3] == link) for link in robot.finger_links]
            others = sorted({(c[2], c[4]) for c in p.getContactPoints(bodyA=env.object_id, physicsClientId=client)
                             if not (c[2] == env.robot_id and c[4] in robot.finger_links)})
            grip_now = grip_point()
            lever_now = lever(obs, grip_now, hand()[1][:, 1])[1] if grip_now is not None else float("nan")
            print(f"  tick {solver.tick} taking the hold: forces {force[0]:.0f} {force[1]:.0f} N, hand points "
                  f"{np.round(hand()[1][:, 0], 2)}, pads' line {np.round(hand()[1][:, 1], 2)}, centre of mass "
                  f"{lever_now:+.0f} deg from down, object z {obs['object_pos'][2]:.3f}, also touches (body, link) {others}")
        continue
    if (solver.tick - began) % 100 == 0 and solver.attempt.carry is not None:
        follower = solver.attempt.carry
        in_hand = hand()[1].T @ rotation(obs["object_xyzw"])
        grip_now = grip_point()
        lever_now = lever(obs, grip_now, hand()[1][:, 1])[1] if grip_now is not None else float("nan")
        print(f"  tick {solver.tick:4d} {solver.state.value:8s} {follower.phase:11s} {follower._along:5.2f} of "
              f"{follower.plan.length:5.2f} m, turned in the hand "
              f"{np.degrees(np.linalg.norm(ik.rotation_vector(in_hand @ start[0].T))):5.1f} deg, centre of mass "
              f"{lever_now:+6.1f} deg from down, {100 * float(info['dist_to_goal']):.0f} cm to go")
print("  hold:", solver.attempt.plan_stats.get("hold"))
# How far each arm link's frame is ahead of the base's centre and to its left, in the pose now held.
base = robot.base_pose(obs["qpos"])
ahead, left = np.array([np.cos(base[2]), np.sin(base[2])]), np.array([-np.sin(base[2]), np.cos(base[2])])
spread = []
for k in robot.arm:
    joint = robot.joint_indices[k]
    at = np.array(p.getLinkState(env.robot_id, joint, computeForwardKinematics=True, physicsClientId=client)[4])
    name = p.getJointInfo(env.robot_id, joint, physicsClientId=client)[12].decode().replace("_link", "")
    spread.append(f"{name} {100 * (at[:2] - base[:2]) @ ahead:+.0f}/{100 * (at[:2] - base[:2]) @ left:+.0f}")
at = np.asarray(obs["object_pos"])
print("  arm, cm ahead/left of the base centre:", ", ".join(spread), f", object {100 * (at[:2] - base[:2]) @ ahead:+.0f}/{100 * (at[:2] - base[:2]) @ left:+.0f}")
print("  maze cell:", np.round(np.sort(np.unique(np.round(np.asarray(env.cube_positions)[:, 0], 3)))[:4], 3))
print("  carry blocked:", solver.attempt.plan_stats.get("carry_blocked"))
print("ended:", solver.state.value, "tick", solver.tick, "delivered", solver.report.delivered_tick,
      "failure:", solver.attempt.failure, "| earlier:", [a["failure"] for a in solver.report.attempts][:3],
      f"| {100 * float(info['dist_to_goal']):.0f} cm from the goal")
solver.close()
close_env(env)
