"""Benchmark the whole trial (pick, carry, reach) over many seeds: headless,
no video, trials in parallel, one small result row per trial.

    pnr/bin/python tools/bench.py OUT --seed-range 1000 1100 -j 30      # 15 objects x 100 seeds
    pnr/bin/python tools/bench.py OUT --seeds 42 17 67 21 --objects 0 12 -j 8
    pnr/bin/python tools/bench.py OUT --resume ...                      # go on after an interruption
    pnr/bin/python tools/bench.py OUT --report                          # print the tables again

A trial is one object on one seed. The seed is the maze's layout seed and
the env's seed, as `run_batch.py --maze-seed S --seed S`. The point cloud
is seeded with it too, so that any trial can be run again alone and gives the
same result:

    pnr/bin/python tools/pick_all.py --batch-maze --maze-seed S --cloud-seed S --deliver --objects N -v --out runs/NAME

`--unseeded-cloud` leaves the cloud and the solver's own draws unseeded, as
in the graded runs; such a trial cannot be repeated.

Each trial is `tools/pick_all.py`'s `run`, in its own process (so it measures
the lift, the hold and the delivery from the simulator, as that tool does).
A trial takes one core, under 2 GB, and one to four minutes; choose -j to
fit the machine. Do not edit the code while a benchmark runs: each trial
imports whatever is on disk. OUT/config.json records the commit.

What is written to OUT (all of it small; copy the folder to collect it):

    config.json     the arguments, the commit, whether the tree had changes
    results.jsonl   one line per trial, appended as each finishes
    results.csv     the same as a table  (written at the end and by --report)
    summary.md      the tables printed at the end
    logs/           the solver's record of each attempt, for the trials that
                    were not delivered or had a drop  (--logs all: every trial)

Points are this project's reading of the rubric in readme.md, out of 40 per
trial: lifted 10 cm within 600 ticks 6, lifted 10 cm 6, held 50 ticks 6,
grasp pose 6 (given with the hold; the error to the planned grasp is in
grasp_error_mm and is counted only if it is over 100 mm), within 10 cm of the
goal 6, within the tick limit 6, no drop after the lift 4.
"""
from __future__ import annotations

import argparse
import ast
import collections
import concurrent.futures as futures
import csv
import json
import os
import socket
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LIFT_TICKS = 600
DROPS = ("object lost during the back-off", "object lost while taking the travel pose",
         "object lost during the carry", "object lost during the reach")
POINTS = ("lift_600", "lift_10cm", "hold_50", "grasp_pose", "reach_10cm", "reach_in_time", "no_drop")


def child(object_idx: int, seed: int, seeded: bool, ticks: int, settings: dict) -> int:
    """One trial in this process; its row is printed as ROW:{json}."""
    from pick_all import run
    row = run(object_idx, seed, seed if seeded else None, ticks, True, True, settings, None, True)
    # The maze this trial wrote for itself (tools/trial.py, make_env): two files in a folder of its own.
    folder = ROOT / "runs" / "tmp" / f"batch_maze_{os.getpid()}_{object_idx}_{seed}"
    for name in ("maze.urdf", "maze.json"):
        (folder / name).unlink(missing_ok=True)
    try:
        folder.rmdir()
    except OSError:
        pass
    print("ROW:" + json.dumps(row), flush=True)
    return 0


def score(row: dict, ticks: int) -> dict:
    """The rubric's points for one trial's row, and the drops they rest on."""
    failures = [f for f in (row.get("failure") or "").split("; ") if f]
    drops = sum(f in DROPS for f in failures)
    lifted = row.get("lift_tick") is not None
    held = bool(row.get("hold_50"))
    error = row.get("grasp_error_mm")
    delivered = row.get("delivered_tick")
    points = dict(
        lift_600=6 * bool(lifted and row["lift_tick"] <= LIFT_TICKS),
        lift_10cm=6 * lifted,
        hold_50=6 * held,
        grasp_pose=6 * bool(held and (error is None or error <= 100.0)),
        reach_10cm=6 * (delivered is not None),
        reach_in_time=6 * bool(delivered is not None and delivered <= ticks),
        no_drop=4 * bool(lifted and drops == 0),
    )
    return dict(drops=drops, points=sum(points.values()), **{f"pts_{k}": v for k, v in points.items()})


def load(out: Path) -> list[dict]:
    path = out / "results.jsonl"
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass                                    # a line cut short by an interruption
    return rows


def report(out: Path) -> str:
    """Write results.csv and summary.md from results.jsonl; returns the summary."""
    rows = sorted(load(out), key=lambda r: (r["seed"], r["object"]))
    if not rows:
        return "no results yet"
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with open(out / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    def line(name: str, group: list[dict]) -> str:
        n = len(group)
        ticks = [r["delivered_tick"] for r in group if r["delivered_tick"] is not None]
        clean = sum(r["delivered_tick"] is not None and r["drops"] == 0 for r in group)
        return (f"| {name} | {n} | {sum(r['lift_tick'] is not None for r in group)} "
                f"| {sum(r['pts_lift_600'] > 0 for r in group)} | {sum(bool(r['hold_50']) for r in group)} "
                f"| {len(ticks)} ({100 * len(ticks) / n:.0f} %) | {clean} ({100 * clean / n:.0f} %) "
                f"| {sum(r['drops'] > 0 for r in group)} | {sum(r['points'] for r in group) / n:.1f} "
                f"| {int(statistics.median(ticks)) if ticks else '-'} |")

    seeds = sorted({r["seed"] for r in rows})
    crashed = sum(str(r.get("failure") or "").startswith("CRASH") for r in rows)
    text = [f"# Benchmark: {len(rows)} trials, {len(seeds)} seeds ({seeds[0]} to {seeds[-1]}), "
            f"{len({r['object'] for r in rows})} objects", ""]
    config = out / "config.json"
    if config.exists():
        c = json.loads(config.read_text())
        text += [f"Commit `{c.get('commit')}`{' with uncommitted changes' if c.get('dirty') else ''}, "
                 f"cloud {'seeded with the trial seed' if c.get('seeded_cloud') else 'unseeded'}, "
                 f"{c.get('ticks')} ticks, settings {c.get('settings') or 'default'}, host {c.get('host')}.", ""]
    head = ("| Object | Trials | Lifted | Lifted by tick 600 | Held 50 | Delivered | Delivered, no drop "
            "| Trials with a drop | Mean points of 40 | Median delivery tick |")
    text += [head, "|" + "---|" * 10]
    for object_idx in sorted({r["object"] for r in rows}):
        group = [r for r in rows if r["object"] == object_idx]
        text.append(line(f"{object_idx} {group[0]['name']}", group))
    text += [line("**all**", rows), ""]
    if crashed:
        text += [f"Trials that crashed or ran out of wall time (counted above as not delivered): {crashed}", ""]

    per_seed = collections.Counter()
    for seed in seeds:
        group = [r for r in rows if r["seed"] == seed]
        per_seed[(sum(r["delivered_tick"] is not None for r in group), len(group))] += 1
    text += ["| Delivered on a seed | Seeds |", "|---|---|"]
    text += [f"| {done} of {n} | {count} |" for (done, n), count in sorted(per_seed.items(), reverse=True)]
    text.append("")

    tally = collections.Counter()
    for r in rows:
        for failure in (r.get("failure") or "").split("; "):
            if failure:
                tally[(r["name"], failure)] += 1
    text += ["| Object | Failure of an attempt | Times |", "|---|---|---|"]
    text += [f"| {name} | {failure} | {count} |" for (name, failure), count in tally.most_common(40)]
    summary = "\n".join(text) + "\n"
    (out / "summary.md").write_text(summary)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", help="folder for the results, e.g. runs/bench_01")
    parser.add_argument("--objects", type=int, nargs="+", default=list(range(15)))
    parser.add_argument("--seeds", type=int, nargs="+", default=None)
    parser.add_argument("--seed-range", type=int, nargs=2, metavar=("FIRST", "END"), default=None,
                        help="seeds FIRST to END - 1")
    parser.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    parser.add_argument("--ticks", type=int, default=3000)
    parser.add_argument("--unseeded-cloud", action="store_true", help="as in the graded runs; not repeatable")
    parser.add_argument("--set", nargs="*", default=[], metavar="NAME=VALUE", help="override solver Config fields")
    parser.add_argument("--logs", choices=("failed", "all", "none"), default="failed")
    parser.add_argument("--wall-limit", type=int, default=3600,
                        help="seconds after which a trial is given up as hung")
    parser.add_argument("--resume", action="store_true", help="run only the trials OUT has no result for")
    parser.add_argument("--report", action="store_true", help="only print the tables for an existing OUT")
    parser.add_argument("--one", type=int, nargs=2, metavar=("OBJECT", "SEED"), default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()

    settings = {}
    for item in args.set:
        name, value = item.split("=", 1)
        try:
            settings[name] = json.loads(value)
        except json.JSONDecodeError:
            settings[name] = ast.literal_eval(value)
    if args.one is not None:
        return child(args.one[0], args.one[1], not args.unseeded_cloud, args.ticks, settings)

    out = Path(args.out).resolve()
    if args.report:
        print(report(out))
        return 0
    seeds = list(args.seeds or [])
    if args.seed_range:
        seeds += list(range(args.seed_range[0], args.seed_range[1]))
    if not seeds:
        parser.error("give --seeds or --seed-range")
    seeds = list(dict.fromkeys(seeds))
    done = {(r["object"], r["seed"]) for r in load(out)}
    if done and not args.resume:
        parser.error(f"{out} already holds {len(done)} results: use --resume, --report or another folder")
    jobs = [(o, s) for s in seeds for o in args.objects if (o, s) not in done]
    out.mkdir(parents=True, exist_ok=True)
    (out / "logs").mkdir(exist_ok=True)

    def git(*command) -> str:
        try:
            return subprocess.run(["git", *command], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        except OSError:
            return ""

    (out / "config.json").write_text(json.dumps(dict(
        commit=git("rev-parse", "--short", "HEAD"),
        dirty=bool(git("status", "--porcelain", "--", "solver.py", "custom_libraries", "tools", "run_batch.py")),
        objects=args.objects, seeds=seeds, ticks=args.ticks, seeded_cloud=not args.unseeded_cloud,
        settings=settings, jobs=args.jobs, host=socket.gethostname(),
        started=time.strftime("%Y-%m-%d %H:%M:%S")), indent=1))

    environment = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    lock = threading.Lock()
    started, finished = time.perf_counter(), [0]
    sink = open(out / "results.jsonl", "a")

    def run(job) -> None:
        object_idx, seed = job
        command = [sys.executable, str(Path(__file__).resolve()), str(out), "--one", str(object_idx), str(seed),
                   "--ticks", str(args.ticks)] + (["--unseeded-cloud"] if args.unseeded_cloud else []) + \
                  (["--set"] + args.set if args.set else [])
        began = time.perf_counter()
        try:
            text = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True, errors="replace",
                                  timeout=args.wall_limit).stdout
            why = "CRASH: no result (see the log)"
        except subprocess.TimeoutExpired as expired:
            text = expired.stdout if isinstance(expired.stdout, str) else (expired.stdout or b"").decode(errors="replace")
            why = f"CRASH: no result after {args.wall_limit} s"
        at = text.rfind("ROW:")
        if at >= 0:
            row = json.loads(text[at + 4:].splitlines()[0])
        else:
            row = dict(object=object_idx, name=f"object {object_idx}", lift_tick=None, hold_50=False,
                       delivered_tick=None, failure=why, grasp_error_mm=None, ticks=0,
                       wall_s=round(time.perf_counter() - began, 1))
        row = dict(seed=seed, **row, **score(row, args.ticks))
        clean = row["delivered_tick"] is not None and row["drops"] == 0
        if args.logs == "all" or (args.logs == "failed" and not clean):
            (out / "logs" / f"o{object_idx:02d}_s{seed}.log").write_text(text)
        with lock:
            sink.write(json.dumps(row) + "\n")
            sink.flush()
            finished[0] += 1
            spent = time.perf_counter() - started
            left = spent / finished[0] * (len(jobs) - finished[0])
            print(f"[{finished[0]:5d}/{len(jobs)}] o{object_idx:02d} s{seed:<6d} lift@{row['lift_tick']} "
                  f"delivered@{row['delivered_tick']} drops {row['drops']} pts {row['points']:2d}  "
                  f"{row.get('wall_s')} s   left {left / 60:.0f} min   {(row.get('failure') or '')[:70]}", flush=True)

    print(f"{len(jobs)} trials ({len(done)} already done), {args.jobs} at a time, results in {out}", flush=True)
    pool = futures.ThreadPoolExecutor(args.jobs)
    try:
        list(pool.map(run, jobs))
    except KeyboardInterrupt:
        pool.shutdown(wait=False, cancel_futures=True)
        print("\ninterrupted: the finished trials are kept; go on with --resume", flush=True)
    finally:
        sink.close()
    print("\n" + report(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
