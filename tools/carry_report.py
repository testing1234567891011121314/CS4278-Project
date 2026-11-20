"""Where a run of `tools/pick_all.py --deliver` loses its objects.

Reads OUT/results.csv and the per-object logs of one run (the course's 15 or
a library, see tools/ycb_library.py) and prints: how far the objects got, what
ended every attempt, when in the carry objects were lost, how the carries
split by what can be read off each attempt (speed, mass, grasp width, depth in
the hand, hand direction, hold), and what happened after a drop.

    pnr/bin/python tools/carry_report.py runs/library_01 [--objects assets/ycb_extra]

Every number is a count of attempts in that one run: one maze, one cloud draw
per object. It shows where to look, not a success rate.
"""
from __future__ import annotations

import argparse
import ast
import collections
import csv
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LOST = "object lost during the carry"


def read(run: Path, objects: Path) -> tuple[list, list]:
    """(objects, carries) of a run: one dict per object, one per attempt that planned a carry."""
    rows = {r["name"]: r for r in csv.DictReader(open(run / "results.csv"))}
    things, carries = [], []
    for path in sorted(run.glob("*.log")):
        name = path.stem[3:]
        if name not in rows:
            continue
        text = path.read_text(errors="replace")
        found = re.search(r"^transitions: (\[.*\])\s*$", text, re.M)
        transitions = ast.literal_eval(found.group(1)) if found else []
        attempts = re.findall(r"^attempt: \{(.*)\}\s*$", text, re.M)
        folder = [d for d in objects.iterdir() if d.name == name or d.name[4:] == name]
        mass = None
        if folder:
            urdf = (folder[0] / "coacd_decomposed_object_one_link.urdf").read_text()
            mass = float(re.search(r'mass value="([^"]+)"', urdf).group(1))
        failures = []
        spans = [(t, transitions[k + 1][0]) for k, (t, s) in enumerate(transitions[:-1]) if s == "navigate"]
        k = 0
        for a in attempts:
            fail = re.match(r"failure: ([^,]*?)(,|$)", a)
            fail = re.sub(r"\(.*", "", fail.group(1)).strip() if fail else None
            failures.append(fail)
            if "carry: {" not in a and "carry_blocked" not in a and "hold: {" not in a:
                continue
            span = spans[k] if k < len(spans) else None
            k += 1
            plan = re.search(r"carry: \{([^}]*)\}", a)
            if plan is None or span is None:
                continue

            def number(key, where=a):
                m = re.search(key + r": (-?[\d.]+)", where)
                return float(m.group(1)) if m else None
            approach = re.search(r"approach: \[([^\]]*)\]", a)
            hold = re.search(r"hold: \{([^}]*)\}", a)
            hold = hold.group(1) if hold else ""
            carries.append(dict(
                name=name, mass=mass, fail=fail, ticks=span[1] - span[0], start=span[0],
                delivered=fail is None and bool(rows[name]["delivered_tick"]),
                step=number("step_mm", plan.group(1)), turn=abs(number("turn_deg", plan.group(1)) or 0.0),
                steered="steered: true" in plan.group(1), width=number("width_mm"), offset=number("offset_mm"),
                hand_z=float(approach.group(1).split(",")[2]) if approach else None,
                floor="pick_mode: floor" in a,
                hold="laid level" if "laid_deg" in hold and "not_laid" not in hold else
                     "rolled level" if "rolled_deg" in hold else "kept as lifted" if "kept" in hold else
                     "folded" if "spread_cm" in hold else "none"))
        things.append(dict(name=name, row=rows[name], transitions=transitions, failures=failures, mass=mass))
    return things, carries


def table(title: str, carries: list, key) -> None:
    groups = collections.defaultdict(list)
    for c in carries:
        k = key(c)
        if k is not None:
            groups[k].append(c)
    print(f"\n{title}")
    print(f"  {'':30s} {'carries':>7s} {'delivered':>9s} {'lost':>5s}  {'median ticks to the loss':>24s}")
    for k in sorted(groups):
        g = groups[k]
        lost = [c["ticks"] for c in g if c["fail"] == LOST]
        print(f"  {k[2:] if k[1:2] == ' ' else k:30s} {len(g):7d} {sum(c['delivered'] for c in g):9d} {len(lost):5d}  "
              f"{int(np.median(lost)) if lost else '-':>24}")


def band(value, edges, labels):
    if value is None:
        return None
    for index, edge in enumerate(edges):
        if value < edge:
            return f"{chr(97 + index)} {labels[index]}"
    return f"{chr(97 + len(edges))} {labels[-1]}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run", help="folder of one pick_all.py --deliver run")
    parser.add_argument("--objects", default=None, help="folder of the objects (default: guessed from the run's names)")
    args = parser.parse_args()
    run = ROOT / args.run
    objects = ROOT / (args.objects or ("assets/ycb_extra" if any(p.stem[3:4].isdigit() for p in run.glob("*.log"))
                                       else "assets/ycb_objects"))
    things, carries = read(run, objects)

    def got_to(t, state):
        return any(s == state for _, s in t["transitions"])
    print(f"{run.name}: {len(things)} objects")
    for label, count in (("reached GRASP", sum(got_to(t, "grasp") for t in things)),
                         ("lifted 10 cm with both pads on", sum(bool(t["row"]["lift_tick"]) for t in things)),
                         ("held 50 ticks", sum(t["row"]["hold_50"] == "True" for t in things)),
                         ("reached NAVIGATE", sum(got_to(t, "navigate") for t in things)),
                         ("reached REACH", sum(got_to(t, "reach") for t in things)),
                         ("delivered", sum(bool(t["row"]["delivered_tick"]) for t in things))):
        print(f"  {label:32s} {count:3d}")

    reasons = collections.Counter(f for t in things for f in t["failures"] if f)
    print("\nWHAT ENDED EACH ATTEMPT (all attempts of all objects)")
    for reason, count in reasons.most_common():
        print(f"  {count:4d}  {reason}")

    lost = [c for c in carries if c["fail"] == LOST]
    print(f"\nCARRIES: {len(carries)} planned, {sum(c['delivered'] for c in carries)} delivered, {len(lost)} lost")
    print("\nTICKS FROM ENTERING NAVIGATE TO THE LOSS")
    print("  (the hold, the torso's rise, the wrist rolls and the turn on the spot take about the first 200-350)")
    edges = [0, 100, 200, 300, 400, 600, 1000, 1500, 3001]
    for lo, hi in zip(edges[:-1], edges[1:]):
        count = sum(lo <= c["ticks"] < hi for c in lost)
        print(f"  {lo:5d}-{hi - 1:<5d} {count:3d}  " + "#" * count)

    table("BY THE CARRY'S SPEED", carries,
          lambda c: band(c["step"], [10.01, 12.01, 16.01, 22.01], ["up to 10 mm per tick", "10-12", "12-16", "16-22", "over 22"]))
    table("BY WHEN THE CARRY STARTED", carries,
          lambda c: band(c["start"], [600, 1500], ["before tick 600", "tick 600-1,500", "after tick 1,500"]))
    table("BY MASS", carries,
          lambda c: band(c["mass"], [0.05, 0.2, 0.6], ["under 0.05 kg", "0.05-0.2 kg", "0.2-0.6 kg", "over 0.6 kg"]))
    table("BY THE GRASP'S WIDTH", carries,
          lambda c: band(c["width"], [20, 50, 85], ["under 20 mm", "20-50 mm", "50-85 mm", "over 85 mm (hand nearly full)"]))
    table("BY DEPTH IN THE HAND (where on the pads the contact is)", carries,
          lambda c: band(c["offset"], [8.01, 24.01], ["0-8 mm from the pads' back", "12-24 mm", "beyond 24 mm"]))
    table("BY THE HAND'S DIRECTION AT THE GRASP", carries,
          lambda c: band(c["hand_z"], [-0.95, -0.5], ["straight down", "tilted", "level or pointing up"]))
    table("BY WRIST STEERING", carries, lambda c: "a steered" if c["steered"] else "b not steered")
    table("BY THE HOLD TAKEN", carries, lambda c: c["hold"])
    table("BY THE TURN ON THE SPOT THAT STARTS THE CARRY", carries,
          lambda c: band(c["turn"], [45, 90, 181], ["under 45 degrees", "45-90", "90-180", "over 180"]))
    table("FROM THE TABLE, OR AFTER A RE-PICK FROM THE FLOOR", carries,
          lambda c: "b picked up from the floor" if c["floor"] else "a from the table")

    after = collections.Counter()
    for t in things:
        for now, then in zip(t["failures"][:-1], t["failures"][1:]):
            if now == LOST:
                after[then or "picked up again, and delivered or still being carried at the end"] += 1
    print("\nWHAT THE NEXT ATTEMPT DID AFTER A DROP IN THE CARRY")
    for reason, count in after.most_common():
        print(f"  {count:4d}  {reason}")

    print("\nNEVER LIFTED")
    for t in things:
        if not t["row"]["lift_tick"]:
            first = next((f for f in t["failures"] if f), "-")
            print(f"  {t['name']:28s} {t['mass'] if t['mass'] is None else round(t['mass'], 3)} kg   first: {first}"
                  f"   most often: {collections.Counter(f for f in t['failures'] if f).most_common(1)[0][0] if any(t['failures']) else '-'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
