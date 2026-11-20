"""Survey of travel poses (docs/plan.md, "Segment 3 from here", step 3).

For each object: pick it once with the solver, save the simulator's state at
the end of the back-off, and from that same state try every pose of
`custom_libraries.navigation.travel_pose.SURVEY`. Each pose gets the same
stimulus, built from `navigation/carry.py`: the turn on the spot and the first
2.5 m of the route to the goal, at a carry's speed. (The solver itself has no
carry at present: after the back-off it holds its pose, and the survey takes
over from there.) The grasp is whatever
the pick made; only the hand's direction and the arm's fold differ.

    pnr/bin/python tools/travel_poses.py [--objects 0 1 ...] [--cloud-seed 1] [--maze-seed 42]
                                         [--step-mm 11] [--out runs/travel_01]

Saving and restoring the simulator's state is test equipment: the solver
itself never writes to the simulator (R30).

Columns of OUT/results.csv, one row per object and pose:
  reached      the arm could take the pose and the way into it was clear
  lost_in      where the object was lost: "pose" (while the pose was taken), a
               phase of the carry ("roll across", "turn", "roll along",
               "drive"), or empty if it was still held at the end
  driven_m     how far the object was led along the route before the end or the loss
  slip_*_mm    how far the object moved in the hand during the stimulus: along
               the fingers, along the pads' line, across both
  turn_*_deg   how far it turned in the hand, about the same three axes
  hand_z, lever_mm, lever_deg   the grasp as picked: the hand's axis (z), and
               where the centre of mass lies from the pads' line
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from trial import ROOT, close_env, make_env, object_names, quiet

ROUTE_CLEARANCES = (0.42, 0.38, 0.34)   # distances the stimulus's route keeps from walls, in the order tried, metres
POSE_RATE = 0.03                        # fastest any arm joint moves while a pose is taken, rad per tick
SETTLE_TICKS = 20                       # ticks waited after a pose is taken

FIELDS = ["object", "name", "pose", "reached", "lost_in", "driven_m", "ticks", "slip_fingers_mm", "slip_pads_mm",
          "slip_across_mm", "turn_deg", "turn_fingers_deg", "turn_pads_deg", "turn_across_deg", "least_force_N",
          "arm_touched_wall", "hand_z", "lever_mm", "lever_deg", "mass_kg", "pick_tick", "note"]


def survey(object_idx: int, maze_seed: int, cloud_seed: int, step: float, distance: float, batch_maze: bool) -> list:
    import pybullet as p
    from custom_libraries.common import Status
    from custom_libraries.grasping import ik
    from custom_libraries.grasping.executor import PathStepper, is_held
    from custom_libraries.navigation import carry
    from custom_libraries.navigation.grid import OccupancyGrid
    from custom_libraries.navigation.travel_pose import SURVEY, fold_path, hand_rotation
    from custom_libraries.planning_sim import rotation
    from solver import Config, Solver, State

    name = object_names()[object_idx]
    env = make_env(object_idx, maze_seed, cloud_seed, batch_maze=batch_maze)
    client = env.pb_physics_client
    solver = Solver(env, Config(seed=cloud_seed))
    rows = []

    def blank(pose: str, **more) -> dict:
        return dict({k: None for k in FIELDS}, object=object_idx, name=name, pose=pose, **more)

    try:
        obs = env._get_obs()
        while solver.tick < 900 and solver.state is not State.NAVIGATE:
            if solver.state in (State.DONE, State.STOPPED):
                break
            with quiet():
                obs, _ = env.step(solver.get_action())
        if solver.state is not State.NAVIGATE:
            failures = [a["failure"] for a in solver.report.attempts] + [solver.attempt.failure]
            return [blank("(no pick)", note="; ".join(str(f) for f in failures if f)[:200])]

        scene, robot = solver.scene, solver.scene.robot
        hold = solver._hold.copy()
        saved = p.saveState(physicsClientId=client)
        pick_tick = solver.tick
        mass = p.getDynamicsInfo(env.object_id, -1, physicsClientId=client)[0]

        def hand():
            state = p.getLinkState(env.robot_id, robot.gripper_link, computeForwardKinematics=True,
                                   physicsClientId=client)
            return np.array(state[4]), rotation(state[5])

        def grip_point():
            points = [(np.array(c[6]), c[9]) for c in
                      p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client)
                      if c[3] in robot.finger_links and c[9] > 0]
            weight = sum(f for _, f in points)
            return (sum(x * f for x, f in points) / weight, weight) if weight > 0 else (None, 0.0)

        def in_hand(obs):
            position, turn = hand()
            return turn.T @ (np.asarray(obs["object_pos"], dtype=float) - position), turn.T @ rotation(obs["object_xyzw"])

        # The grasp as picked: the hand's axis, and the centre of mass from the pads' line.
        position, turn = hand()
        grip, _ = grip_point()
        if grip is None:
            return [blank("(no pick)", note="no contact with the pads after the back-off")]
        centre = np.asarray(obs["object_pos"], dtype=float)
        lever = centre - grip
        lever = lever - (lever @ turn[:, 1]) * turn[:, 1]
        down = np.array([0.0, 0.0, -1.0]) + turn[2, 1] * turn[:, 1]
        lever_deg = float(np.degrees(np.arccos(np.clip(lever @ down / max(np.linalg.norm(lever) * np.linalg.norm(down),
                                                                             1e-9), -1.0, 1.0))))
        base = robot.base_pose(hold)
        forward = np.array([np.cos(base[2]), np.sin(base[2]), 0.0])

        # The stimulus: the start of the route the carry would take.
        grids = [OccupancyGrid.from_world(scene.world, solver.config.grid_resolution, keep, obstacle_radius=keep)
                 for keep in ROUTE_CLEARANCES]
        routes = carry.plan_route(grids, base[:2], scene.world.goal[:2], carry.CarrySettings())
        if not routes:
            return [blank("(no route)", note="no route from the back-off to the goal")]
        route = routes[0]
        marks = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))])
        length = min(marks[-1], 0.75 + distance)
        keep = marks < length
        route = np.vstack([route[keep], [np.interp(length, marks, route[:, 0]), np.interp(length, marks, route[:, 1])]])

        for pose in SURVEY:
            p.restoreState(stateId=saved, physicsClientId=client)
            obs = env._get_obs()
            row = blank(pose.name, hand_z=round(float(turn[2, 0]), 2), lever_mm=round(1000 * float(np.linalg.norm(lever))),
                        lever_deg=round(lever_deg), mass_kg=round(mass, 3), pick_tick=pick_tick, reached=True,
                        lost_in="", ticks=0, least_force_N=None, arm_touched_wall=False)
            rows.append(row)
            action = hold.copy()
            lost = 0

            def tick(action) -> bool:
                """One env step; False once the object has been off the pads four ticks running."""
                nonlocal obs, lost
                with quiet():
                    obs, _ = env.step(action)
                row["ticks"] += 1
                force = grip_point()[1]
                row["least_force_N"] = round(force if row["least_force_N"] is None else min(row["least_force_N"], force))
                if any(c[2] not in (env.robot_id, env.object_id) and c[3] in scene.collision.arm_links
                       for c in p.getContactPoints(bodyA=env.robot_id, physicsClientId=client)):
                    row["arm_touched_wall"] = True
                lost = 0 if is_held(env, robot) else lost + 1
                return lost < 4

            # Take the pose.
            if pose.fold:
                spin = hand_rotation(pose, turn, forward)
                path, stats = fold_path(scene, solver.rng, hold, grip, centre, obs["object_xyzw"], spin, pose.reach)
                row["note"] = json.dumps(stats, default=str).replace('"', "")[:200]
                if len(path) < 2:
                    row["reached"] = False
                    continue
                limit = np.full(len(hold), POSE_RATE)
                limit[robot.torso] = 0.002
                stepper = PathStepper(path, limit)
                held = True
                while held and not stepper.finished:
                    action = stepper.step()
                    held = tick(action)
                for _ in range(SETTLE_TICKS):
                    held = held and tick(action)
                if not held:
                    row["lost_in"] = "pose"
                    continue

            # The stimulus.
            _, now = hand()
            upright = abs(now[2, 0]) >= carry.CarrySettings().upright
            closing = float(np.arctan2(now[1, 1], now[0, 1])) + (np.pi / 2.0 if pose.steer == "across" else 0.0) \
                if upright and pose.steer != "none" else None
            base_now = robot.base_pose(action)
            plan = carry.make_plan(route, base_now, np.asarray(obs["object_pos"])[:2], closing,
                                   action[robot.arm[-1]], int(np.sign(now[2, 0])), step)
            follower = carry.CarryFollower(plan, robot, action, 0.0, plan.turn)
            start = in_hand(obs)
            last = start
            status = Status.RUNNING
            while status is Status.RUNNING and row["ticks"] < 1500:
                action, status = follower.step(robot.base_pose(obs["qpos"]))
                if not tick(action):
                    row["lost_in"] = follower.phase
                    break
                if is_held(env, robot):
                    last = in_hand(obs)
            if status is Status.FAILED and not row["lost_in"]:
                row["note"] = ((row["note"] or "") + " carry: " + str(follower.failure))[:200]
            row["driven_m"] = round(float(follower._along - plan.arm), 2)
            slip = 1000 * (last[0] - start[0])
            turned = np.degrees(ik.rotation_vector(last[1] @ start[1].T))
            row.update(slip_fingers_mm=round(float(slip[0]), 1), slip_pads_mm=round(float(slip[1]), 1),
                       slip_across_mm=round(float(slip[2]), 1), turn_deg=round(float(np.linalg.norm(turned)), 1),
                       turn_fingers_deg=round(float(turned[0]), 1), turn_pads_deg=round(float(turned[1]), 1),
                       turn_across_deg=round(float(turned[2]), 1))
        return rows
    except Exception:
        rows.append(blank("(crash)", note=traceback.format_exc().strip().splitlines()[-1][:200]))
        traceback.print_exc()
        return rows
    finally:
        solver.close()
        close_env(env)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--objects", type=int, nargs="+", default=list(range(15)))
    parser.add_argument("--maze-seed", type=int, default=42)
    parser.add_argument("--cloud-seed", type=int, default=1)
    parser.add_argument("--step-mm", type=float, default=11.0, help="the object's speed in the stimulus, mm per tick")
    parser.add_argument("--distance", type=float, default=2.5, help="metres of route driven in the stimulus")
    parser.add_argument("--env-maze", action="store_true", help="the env's own maze instead of run.sh's")
    parser.add_argument("--out", default="runs/travel_poses")
    parser.add_argument("--one", type=int, default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.one is not None:
        rows = survey(args.one, args.maze_seed, args.cloud_seed, args.step_mm / 1000.0, args.distance, not args.env_maze)
        print("ROWS:" + json.dumps(rows), flush=True)
        return 0

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    names, rows, started = object_names(), [], time.perf_counter()
    for object_idx in args.objects:
        log = out / f"{object_idx:02d}_{names[object_idx]}.log"
        with open(log, "w") as sink:
            subprocess.run([sys.executable, __file__, "--one", str(object_idx), "--maze-seed", str(args.maze_seed),
                            "--cloud-seed", str(args.cloud_seed), "--step-mm", str(args.step_mm),
                            "--distance", str(args.distance)] + (["--env-maze"] if args.env_maze else []),
                           stdout=sink, stderr=subprocess.STDOUT)
        text = log.read_text(errors="replace")
        at = text.rfind("ROWS:")
        found = json.loads(text[at + 5:].splitlines()[0]) if at >= 0 else \
            [dict({k: None for k in FIELDS}, object=object_idx, name=names[object_idx], pose="(crash)",
                  note="no result (see the log)")]
        rows += found
        for r in found:
            print(f"{r['name']:16s} {r['pose']:24s} " +
                  ("not reachable" if r["reached"] is False else
                   f"lost in {r['lost_in']:11s} after {r['driven_m']} m" if r["lost_in"] else
                   f"held  {r['driven_m']} m  slip {r['slip_fingers_mm']}/{r['slip_pads_mm']}/{r['slip_across_mm']} mm  "
                   f"turned {r['turn_deg']} deg" if r["driven_m"] is not None else str(r["note"])), flush=True)
    with open(out / "results.csv", "w", newline="") as sink:
        writer = csv.DictWriter(sink, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    (out / "settings.json").write_text(json.dumps(vars(args)))
    print(f"\n{len(rows)} rows in {out / 'results.csv'}   ({time.perf_counter() - started:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
