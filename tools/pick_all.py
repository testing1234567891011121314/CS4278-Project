"""Run the pick (go to object, grasp, back off) for several objects on one maze.

    pnr/bin/python tools/pick_all.py [--objects 0 1 ...] [--maze-seed 42] [--cloud-seed 300]
                                     [--ticks 900] [--out runs/pick_all] [-v]

Everything in the table is measured from the simulator, not taken from the
solver: where the object rested, the tick at which it was 10 cm up with both
pads on it, whether it stayed so for 50 ticks, and whether it was still held
when the back-off ended. Each object runs in its own process (memory is not
given back between trials in one process), with its output in OUT/NAME.log.
Run under the memory cap:

    flock /home/shared/pnr_sim.lock systemd-run --user --scope --quiet -p MemoryMax=2000M \\
        -p MemorySwapMax=0 pnr/bin/python tools/pick_all.py
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

from trial import OBJECT_NAMES, ROOT, close_env, make_env, quiet

LIFT, HOLD = 0.10, 50


def run(object_idx: int, maze_seed: int, cloud_seed: int, ticks: int, verbose: bool) -> dict:
    import pybullet as p
    from solver import Config, Solver, State

    row = dict(object=object_idx, name=OBJECT_NAMES[object_idx], rest_z=None, lift_tick=None, max_lift_cm=None,
               hold_50=False, held_at_end=False, backed_off_m=None, state=None, attempts=0, failure=None,
               grasp_error_mm=None, errors=0, ticks=0, wall_s=None)
    started = time.perf_counter()
    env = make_env(object_idx, maze_seed, cloud_seed)
    solver = None
    try:
        client = env.pb_physics_client
        fingers = [j for j in range(p.getNumJoints(env.robot_id, physicsClientId=client))
                   if b"finger" in p.getJointInfo(env.robot_id, j, physicsClientId=client)[12]]
        solver = Solver(env, Config(seed=cloud_seed))

        def held() -> bool:
            points = p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client)
            return set(fingers) <= {c[3] for c in points if c[9] > 0.5}

        last, still, rest_z, held_run, low, base_at_lift = None, 0, None, 0, None, None
        done_for = 0
        for tick in range(ticks):
            with quiet():
                obs, _ = env.step(solver.get_action())
            z = float(obs["object_pos"][2])
            position = np.asarray(obs["object_pos"], dtype=float)
            if row["lift_tick"] is None:
                # The resting height is the last height at which the object lay still, untouched.
                still = still + 1 if last is not None and np.linalg.norm(position - last) < 5e-4 else 0
                if still >= 10 and not held():
                    rest_z = z
                elif rest_z is None and tick == 130:
                    rest_z = z
            last = position
            holding = held()
            if rest_z is not None:
                if row["lift_tick"] is None and holding and z - rest_z >= LIFT:
                    row["lift_tick"], held_run, low = tick + 1, 0, z - rest_z
                    base_at_lift = np.asarray(obs["qpos"], dtype=float)[:2].copy()
                elif row["lift_tick"] is not None and held_run < HOLD:
                    if holding:
                        held_run += 1
                        low = min(low, z - rest_z)
                        row["hold_50"] = held_run >= HOLD and low >= LIFT
                    else:
                        held_run = HOLD + 1            # the hold was broken inside the window
                if row["lift_tick"] is not None:
                    row["max_lift_cm"] = round(100 * max(z - rest_z, (row["max_lift_cm"] or 0) / 100), 1)
            row["ticks"] = tick + 1
            if solver.state in (State.NAVIGATE, State.DONE, State.STOPPED):
                done_for += 1
                if done_for >= 20:
                    break
        row["rest_z"] = None if rest_z is None else round(rest_z, 3)
        row["held_at_end"] = bool(held())
        if base_at_lift is not None:
            row["backed_off_m"] = round(float(np.linalg.norm(np.asarray(obs["qpos"], dtype=float)[:2] - base_at_lift)), 2)
        row["state"] = solver.state.value
        attempts = solver.report.attempts + [dict(failure=solver.attempt.failure, search=solver.attempt.search,
                                                  plan_stats=solver.attempt.plan_stats,
                                                  grasp_error=solver.attempt.grasp_error,
                                                  pick_mode=solver.attempt.pick_mode)]
        row["attempts"] = len(attempts)
        row["failure"] = "; ".join(str(a["failure"]) for a in attempts if a["failure"]) or None
        if solver.attempt.grasp_error:
            row["grasp_error_mm"] = round(1000 * solver.attempt.grasp_error[0], 1)
        row["errors"] = len(solver.report.errors)
        if verbose:
            print(f"\n--- {row['name']} ---")
            print("transitions:", solver.report.transitions)
            for a in attempts:
                brief = {k: a.get(k) for k in ("failure", "pick_mode", "search", "plan_stats") if a.get(k)}
                print("attempt:", json.dumps(brief, default=str).replace('"', ""))
            for tick, state, text in solver.report.errors:
                print(f"error at tick {tick} in {state}:\n{text}")
    except Exception:
        row["failure"] = "CRASH: " + traceback.format_exc().strip().splitlines()[-1]
        if verbose:
            traceback.print_exc()
    finally:
        if solver is not None:
            solver.close()
        close_env(env)
    row["wall_s"] = round(time.perf_counter() - started, 1)
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--objects", type=int, nargs="+", default=list(range(15)))
    parser.add_argument("--maze-seed", type=int, default=42)
    parser.add_argument("--cloud-seed", type=int, default=300)
    parser.add_argument("--ticks", type=int, default=900)
    parser.add_argument("--out", default="runs/pick_all")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--one", type=int, default=None, help=argparse.SUPPRESS)   # the child process's object
    args = parser.parse_args()
    if args.one is not None:
        print("ROW:" + json.dumps(run(args.one, args.maze_seed, args.cloud_seed, args.ticks, True)), flush=True)
        return 0

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for object_idx in args.objects:
        log = out / f"{object_idx:02d}_{OBJECT_NAMES[object_idx]}.log"
        with open(log, "w") as sink:
            subprocess.run([sys.executable, __file__, "--one", str(object_idx), "--maze-seed", str(args.maze_seed),
                            "--cloud-seed", str(args.cloud_seed), "--ticks", str(args.ticks)],
                           stdout=sink, stderr=subprocess.STDOUT)
        text = log.read_text(errors="replace")
        at = text.rfind("ROW:")
        if at < 0:                      # the child died without a result: count it, do not drop it
            row = dict(object=object_idx, name=OBJECT_NAMES[object_idx], rest_z=None, lift_tick=None,
                       max_lift_cm=None, hold_50=False, held_at_end=False, backed_off_m=None, state=None,
                       attempts=0, failure="CRASH: no result (see the log)", grasp_error_mm=None, errors=0,
                       ticks=0, wall_s=None)
        else:
            row = json.loads(text[at + 4:].splitlines()[0])
        rows.append(row)
        if args.verbose:
            lines = text.splitlines()
            tries = [line for line in lines if line.startswith("attempt:")]
            shown = tries if len(tries) <= 3 else tries[:2] + [f"... {len(tries) - 3} more ..."] + tries[-1:]
            keep = ("transitions:", "error at tick", "Traceback", "  File")
            print("\n".join(line[:420] for line in lines if line.startswith(keep) or "Error:" in line))
            print("\n".join(line[:420] for line in shown))
        print(f"{row['name']:16s} lift@{row['lift_tick']} max {row['max_lift_cm']} cm  hold50={row['hold_50']}  "
              f"held_end={row['held_at_end']}  back-off {row['backed_off_m']} m  state={row['state']}  "
              f"attempts={row['attempts']}  {row['wall_s']} s  {(row['failure'] or '')[:160]}", flush=True)

    with open(out / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    picked = sum(r["hold_50"] and r["held_at_end"] and r["state"] == "navigate" for r in rows)
    print(f"\nlifted, held 50 ticks and backed off with the object: {picked}/{len(rows)}   "
          f"(maze seed {args.maze_seed}, cloud seed {args.cloud_seed})   results: {out / 'results.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
