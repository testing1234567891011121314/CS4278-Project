"""Run tools/score.py over objects x seeds from a frozen copy of the repo.

Usage: python tools/batch.py OUT_DIR [--objects 5 6 7 14 2] [--seeds 42 7]
                             [-j JOBS] [--max-steps 15000] [--repeat N]

Copies the working tree (without .git, pnr/ and caches) to OUT_DIR/snap, so
edits made while the batch runs do not mix code versions, runs the trials in
parallel with the current interpreter, keeps one log per trial in OUT_DIR and
prints a markdown table plus totals. Re-run with only OUT_DIR --report to
print the table again.
"""
import argparse
import concurrent.futures as cf
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
    shutil.rmtree(snap, ignore_errors=True)
    # assets/ (150 MB) never changes: link it into the snapshot when the
    # platform allows, copy it otherwise.
    skip = shutil.ignore_patterns(".git", ".claude", "pnr", "__pycache__", "pristine", "*.mp4", "*.jsonl", "imgs",
                                  "assets", "runs", "paper_review")

    def ignore(folder, names):
        # OUT_DIR may be inside the repo: never copy it into its own snapshot.
        return set(skip(folder, names)) | {n for n in names if os.path.abspath(os.path.join(folder, n)) == out}

    shutil.copytree(repo, snap, ignore=ignore)
    try:
        os.symlink(os.path.join(repo, "assets"), os.path.join(snap, "assets"), target_is_directory=True)
    except OSError:
        shutil.copytree(os.path.join(repo, "assets"), os.path.join(snap, "assets"))
    jobs = [(o, s, k) for k in range(args.repeat) for o in args.objects for s in args.seeds]
    with cf.ThreadPoolExecutor(args.jobs) as pool:
        list(pool.map(run, jobs))

logs = sorted(f for f in os.listdir(out) if f.endswith(".log"))
rows, crashed = [], []
for f in logs:
    r = load(os.path.join(out, f))
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
