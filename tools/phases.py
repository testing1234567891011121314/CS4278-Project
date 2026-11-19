"""Step breakdown by phase for a tools/batch.py output directory.

Usage: python tools/phases.py OUT_DIR [OUT_DIR ...]

Reads the SCORE lines and their state transitions and prints, per trial and as
medians over delivered trials, the env steps spent in: pick (up to the end of
the lift), hold (state 3), retreat (state 5), carry setup (fixed carry: from
state 67 until the base starts its route is not separable, so the whole of
state 67 is "carry"; yaw carry: the wrist rolls and the turn on the spot),
carry (driving), reach (state 68), and the total to delivery.
"""
import json
import os
import sys

import numpy as np


def load(log):
    text = open(log, errors="replace").read()
    at = text.rfind("SCORE:")
    return json.loads(text[at + 6:].splitlines()[0]) if at >= 0 else None


def breakdown(r):
    end = r["success_step"] or r["steps"]
    marks = r["states"] + [[end, "end"]]
    out = dict(pick=0, hold=0, retreat=0, setup=0, carry=0, reach=0)
    for (start, label), (stop, _) in zip(marks, marks[1:]):
        n = stop - start
        label = str(label)
        if label in ("999", "0", "1", "2"):
            out["pick"] += n
        elif label == "3":
            out["hold"] += n
        elif label == "5":
            out["retreat"] += n
        elif label in ("67:spin_roll", "67:spin", "67:drive_roll"):
            out["setup"] += n
        elif label.startswith("67"):
            out["carry"] += n
        elif label == "68":
            out["reach"] += n
    out["total"] = end
    return out


for out_dir in sys.argv[1:]:
    rows = [r for r in (load(os.path.join(out_dir, f)) for f in sorted(os.listdir(out_dir)) if f.endswith(".log")) if r]
    print(f"\n{out_dir}: {sum(r['success_step'] is not None for r in rows)}/{len(rows)} delivered")
    keys = ["pick", "hold", "retreat", "setup", "carry", "reach", "total"]
    print("| object | seed | " + " | ".join(keys) + " | result |")
    print("|---" * (len(keys) + 3) + "|")
    done = []
    for r in sorted(rows, key=lambda r: (r["object"], -r["seed"])):
        b = breakdown(r)
        res = "delivered" if r["success_step"] else (f"drop st{r['drop']['state']} @{r['drop']['step']}" if r["drop"] else r["stopped"])
        print(f"| {r['object']} | {r['seed']} | " + " | ".join(str(b[k]) for k in keys) + f" | {res} |")
        if r["success_step"]:
            done.append(b)
    if done:
        print("| median (delivered) | | " + " | ".join(f"{np.median([b[k] for b in done]):.0f}" for k in keys) + " | |")
