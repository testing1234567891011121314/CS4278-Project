"""One whole trial with a line per tick while the object is in the hand after
the pick (back-off, travel pose, carry, reach): where the object sits in the
hand and what presses on it. For finding out how an object is lost.

    pnr/bin/python tools/trace_trial.py OBJECT --cloud-seed 5 [--maze-seed 42] [--ticks 3000]
                                        [--from-tick 0] [--set NAME=VALUE ...] > runs/NAME.log

Lines start with "T " and hold: tick, state, the carry's phase, metres led,
the base's x y yaw, the object's centre in the hand's frame in millimetres
(along the hand's axis from the gripper frame, along the pads' line, across
both), the angle in degrees the object has turned in the hand since the
first line, the force on each finger and on the palm in newtons, the
largest force between the object and anything but the robot (and where that
contact is), and the largest force between the robot and anything but the
object above the floor. The solver's
transitions and its record of each attempt follow at the end.
"""
from __future__ import annotations

import argparse
import ast
import json

import numpy as np

from trial import close_env, make_env, object_names, quiet


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("object", type=int)
    parser.add_argument("--maze-seed", type=int, default=42)
    parser.add_argument("--cloud-seed", type=int, default=300)
    parser.add_argument("--ticks", type=int, default=3000)
    parser.add_argument("--from-tick", type=int, default=0)
    parser.add_argument("--set", nargs="*", default=[], metavar="NAME=VALUE")
    args = parser.parse_args()
    settings = {}
    for item in args.set:
        name, value = item.split("=", 1)
        try:
            settings[name] = json.loads(value)
        except json.JSONDecodeError:
            settings[name] = ast.literal_eval(value)

    import pybullet as p
    from custom_libraries.planning_sim import rotation
    from solver import Config, Solver, State

    env = make_env(args.object, args.maze_seed, args.cloud_seed, batch_maze=True)
    solver = Solver(env, Config(seed=args.cloud_seed, **settings))
    client = env.pb_physics_client
    print(f"object {args.object} {object_names()[args.object]}, maze seed {args.maze_seed}, "
          f"cloud seed {args.cloud_seed}, settings {settings}")
    print("T tick state phase led_m | base x y yaw | in hand mm: axis pads across | turned deg | "
          "N: finger finger palm | other N (and where) | robot against a wall N")
    first = None
    try:
        for tick in range(args.ticks):
            with quiet():
                obs, info = env.step(solver.get_action())
            state = solver.state
            if state in (State.BACK_OFF, State.NAVIGATE, State.REACH) and tick >= args.from_tick:
                robot = solver.scene.robot
                link = p.getLinkState(env.robot_id, robot.gripper_link, computeForwardKinematics=True,
                                      physicsClientId=client)
                hand, turn = np.array(link[4]), rotation(link[5])
                inside = turn.T @ (np.asarray(obs["object_pos"], dtype=float) - hand)
                facing = turn.T @ rotation(obs["object_xyzw"])
                if first is None:
                    first = facing
                angle = np.degrees(np.arccos(np.clip((np.trace(first.T @ facing) - 1.0) / 2.0, -1.0, 1.0)))
                force = {}
                for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client):
                    force[c[3]] = force.get(c[3], 0.0) + c[9]
                touching = [c for c in p.getContactPoints(bodyA=env.object_id, physicsClientId=client)
                            if c[2] != env.robot_id]
                other = max([c[9] for c in touching], default=0.0)
                where = max(touching, key=lambda c: c[9])[5] if touching else None
                # The robot against anything but the object, above the floor (its wheels stand on it).
                wall = max([c[9] for c in p.getContactPoints(bodyA=env.robot_id, physicsClientId=client)
                            if c[2] not in (env.robot_id, env.object_id) and c[5][2] > 0.02], default=0.0)
                carry = solver.attempt.carry
                base = robot.base_pose(obs["qpos"])
                print(f"T {tick + 1} {state.value} {carry.phase if carry is not None else '-'} "
                      f"{carry.led if carry is not None else 0.0:.2f} | {base[0]:.2f} {base[1]:.2f} {base[2]:.2f} | "
                      f"{1000 * inside[0]:.0f} {1000 * inside[1]:.0f} {1000 * inside[2]:.0f} | {angle:.0f} | "
                      + " ".join(f"{force.get(k, 0.0):.0f}" for k in (*robot.finger_links, robot.gripper_link))
                      + f" | {other:.0f}"
                      + (f" at {where[0]:.2f} {where[1]:.2f} {where[2]:.2f}" if where is not None else "")
                      + f" | {wall:.0f}")
            elif state not in (State.BACK_OFF, State.NAVIGATE, State.REACH):
                first = None
            if bool(info["success"]):
                print(f"delivered at tick {tick + 1}")
                break
            if state in (State.DONE, State.STOPPED):
                break
        print("transitions:", solver.report.transitions)
        for a in solver.report.attempts + [dict(failure=solver.attempt.failure, plan_stats=solver.attempt.plan_stats,
                                                pick_mode=solver.attempt.pick_mode)]:
            print("attempt:", json.dumps({k: a.get(k) for k in ("failure", "pick_mode", "plan_stats") if a.get(k)},
                                         default=str).replace('"', ""))
    finally:
        solver.close()
        close_env(env)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
