"""Run tools/score.py over objects x seeds from a frozen copy of the repo.

Usage: python tools/batch.py OUT_DIR [--objects 5 6 7 14 2] [--seeds 42 7]
                             [-j JOBS] [--max-steps 15000] [--repeat N] [--resume]

Copies the working tree (without .git, pnr/ and caches) to OUT_DIR/snap, so
edits made while the batch runs do not mix code versions, runs the trials in
parallel with the current interpreter, keeps one log per trial in OUT_DIR and
prints a markdown table plus totals. Re-run with only OUT_DIR --report to
print the table again. With --resume an interrupted batch continues from the
snapshot already in OUT_DIR and runs only the trials that have no score yet.
Every run also writes OUT_DIR/results.csv (one row per trial) and prints a
per-object summary (delivered per seed, points, delivery steps).
"""
import argparse
import concurrent.futures as cf
import csv
import json
import os
import shutil
import subprocess
import sys

NAMES = ("master_chef_can sugar_box tomato_soup_can mustard_bottle potted_meat_can banana strawberry "
         "apple bleach_cleanser bowl mug plate large_marker hammer foam_brick").split()

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--objects", type=int, nargs="+", default=[5, 6, 7, 14, 2])
ap.add_argument("--seeds", type=int, nargs="+", default=[42, 7])
ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
ap.add_argument("--max-steps", type=int, default=15000)
ap.add_argument("--repeat", type=int, default=1)
ap.add_argument("--report", action="store_true", help="only print the table for an existing OUT_DIR")
ap.add_argument("--resume", action="store_true", help="keep OUT_DIR/snap and run only the unscored trials")
args = ap.parse_args()

out = os.path.abspath(args.out)
snap = os.path.join(out, "snap")
repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(job):
    o, s, k = job
    log = os.path.join(out, f"o{o}_s{s}_r{k}.log")
    with open(log, "w") as f:
        subprocess.run([sys.executable, os.path.join(snap, "tools", "score.py"), snap, str(o), str(s),
                        str(args.max_steps)], stdout=f, stderr=subprocess.STDOUT)
    return log


def load(log):
    # Bullet's URDF warnings do not end with a newline, so SCORE: may be mid-line.
    text = open(log, errors="replace").read()
    at = text.rfind("SCORE:")
    return json.loads(text[at + 6:].splitlines()[0]) if at >= 0 else None


if not args.report:
    os.makedirs(out, exist_ok=True)
    fresh = not (args.resume and os.path.isdir(snap))
    if fresh:
        shutil.rmtree(snap, ignore_errors=True)
    # assets/ (150 MB) never changes: link it into the snapshot when the
    # platform allows, copy it otherwise.
    skip = shutil.ignore_patterns(".git", ".claude", "pnr", "__pycache__", "pristine", "*.mp4", "*.jsonl", "imgs",
                                  "assets", "runs", "paper_review", "scratch")

    def ignore(folder, names):
        # OUT_DIR may be inside the repo: never copy it into its own snapshot.
        return set(skip(folder, names)) | {n for n in names if os.path.abspath(os.path.join(folder, n)) == out}

    if fresh:
        shutil.copytree(repo, snap, ignore=ignore)
        try:
            os.symlink(os.path.join(repo, "assets"), os.path.join(snap, "assets"), target_is_directory=True)
        except OSError:
            shutil.copytree(os.path.join(repo, "assets"), os.path.join(snap, "assets"))
    jobs = [(o, s, k) for k in range(args.repeat) for o in args.objects for s in args.seeds]
    if args.resume:
        scored = lambda j: os.path.exists(os.path.join(out, "o%d_s%d_r%d.log" % j)) and load(
            os.path.join(out, "o%d_s%d_r%d.log" % j))
        jobs = [j for j in jobs if not scored(j)]
        print(f"resume: {len(jobs)} trials left", flush=True)
    with cf.ThreadPoolExecutor(args.jobs) as pool:
        list(pool.map(run, jobs))

logs = sorted(f for f in os.listdir(out) if f.endswith(".log"))
rows, crashed = [], []
for f in logs:
    r = load(os.path.join(out, f))
    if r:
        r["repeat"] = int(f.rsplit("_r", 1)[1].split(".")[0])
    (rows if r else crashed).append(r or f)
rows.sort(key=lambda r: (args.objects.index(r["object"]) if r["object"] in args.objects else 99, -r["seed"]))
print("| object | seed | lift step | lift cm (min over hold) | hold 50 | grasp err mm | success step | drop | stopped | pts |")
print("|---|---|---|---|---|---|---|---|---|---|")
for r in rows:
    drop = "-" if r["drop"] is None else f"st{r['drop']['state']} @{r['drop']['step']}"
    err = "-" if r["grasp_pos_err"] is None else round(1000 * r["grasp_pos_err"])
    print(f"| {NAMES[r['object']]} ({r['object']}) | {r['seed']} | {r['lift_step']} | {r['lift_cm']} ({r['hold_min_cm']}) "
          f"| {'yes' if r['hold_50'] else 'no'} | {err} | {r['success_step'] or '-'} | {drop} | {r['stopped']} | {r['total']} |")
for f in crashed:
    print(f"| CRASH {f} | | | | | | | | | 0 |")
n = len(rows) + len(crashed)
if n:
    keys = list(rows[0]["points"]) if rows else []
    print(f"\ntrials: {n}   delivered: {sum(r['success_step'] is not None for r in rows)}/{n}"
          f"   mean points: {sum(r['total'] for r in rows) / n:.1f}/40")
    print("   ".join(f"{k}: {sum(r['points'][k] > 0 for r in rows)}/{n}" for k in keys))

# One row per trial, for spreadsheets and plots.
cols = ["object", "name", "seed", "repeat", "lift_step", "lift_cm", "hold_min_cm", "hold_50", "grasp_pos_err",
        "grasp_ang_err", "success_step", "drop_step", "drop_state", "stopped", "final_state", "steps", "total",
        "carry", "err", "wall_s"]
with open(os.path.join(out, "results.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(cols + [f"pts_{k}" for k in keys])
    for r in rows:
        d = dict(r, name=NAMES[r["object"]], drop_step=(r["drop"] or {}).get("step"),
                 drop_state=(r["drop"] or {}).get("state"))
        w.writerow([d.get(c) for c in cols] + [r["points"][k] for k in keys])
    for name in crashed:
        w.writerow([name, "CRASH"])

if rows:
    seeds = sorted({r["seed"] for r in rows})
    print("\n| object | trials | lifted | delivered | " + " | ".join(f"s{s}" for s in seeds) + " | mean pts | delivery steps |")
    print("|---|---|---|---|" + "---|" * len(seeds) + "---|---|")
    for o in sorted({r["object"] for r in rows}):
        rs = [r for r in rows if r["object"] == o]
        ok = [r["success_step"] for r in rs if r["success_step"] is not None]
        per = [f"{sum(r['success_step'] is not None for r in rs if r['seed'] == s)}/{sum(r['seed'] == s for r in rs)}"
               for s in seeds]
        print(f"| {NAMES[o]} ({o}) | {len(rs)} | {sum(r['lift_step'] is not None for r in rs)} | {len(ok)} | "
              + " | ".join(per) + f" | {sum(r['total'] for r in rs) / len(rs):.1f} | "
              + (f"{min(ok)}-{max(ok)}" if ok else "-") + " |")
    print(f"\nobjects delivered at least once: {len({r['object'] for r in rows if r['success_step'] is not None})}"
          f"/{len({r['object'] for r in rows})}")
