"""Compare two benchmarks trial by trial, and read the ablation that
`tools/bench.py --ablate` ran: writes FULL/report.md and prints it.

    pnr/bin/python tools/bench_report.py runs/bench_01 runs/bench_02

BASE and FULL are folders of tools/bench.py run on the same seeds: BASE with
the code before a change, FULL with it. A trial is one object on one seed and
is repeatable, so each trial of FULL has its own counterpart in BASE and the
comparison is paired.

- Delivered, and delivered with no drop: the trials that changed are counted
  each way, with the exact two-sided sign test on them (McNemar's test).
- Differences in rates and in points: a 95 % interval from resampling whole
  seeds (10,000 times), since the 15 objects of a seed share its maze.
- Each part of the carry (bench.COMPONENTS): FULL against FULL with that part
  switched off, on the trials in which the part acted. The other trials run
  the same without it; the check column says how many of a sample did.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench import COMPONENTS, load                                              # noqa: E402

SAME = ("lift_tick", "delivered_tick", "ticks", "failure")                      # a trial that ran the same


def keyed(rows: list[dict]) -> dict:
    return {(r["object"], r["seed"]): r for r in rows}


def delivered(row: dict) -> int:
    return int(row.get("delivered_tick") is not None)


def clean(row: dict) -> int:
    return int(row.get("delivered_tick") is not None and row.get("drops", 0) == 0)


def points(row: dict) -> int:
    return int(row.get("points", 0))


def sign_test(won: int, lost: int) -> float:
    """Exact two-sided probability of a split at least this uneven, were each change as likely either way."""
    n, k = won + lost, min(won, lost)
    if n == 0:
        return 1.0
    return min(1.0, 2.0 * sum(math.comb(n, i) for i in range(k + 1)) / 2.0 ** n)


def interval(per_seed: dict, total: int, draws: int = 10000) -> tuple[float, float]:
    """95 % interval of sum(per_seed) / total when whole seeds are resampled."""
    values = np.array(list(per_seed.values()), dtype=float)
    if len(values) < 2:
        return float("nan"), float("nan")
    rng = np.random.default_rng(0)
    picks = rng.integers(0, len(values), size=(draws, len(values)))
    sums = values[picks].sum(axis=1) / total
    return float(np.percentile(sums, 2.5)), float(np.percentile(sums, 97.5))


def difference(pairs: list[tuple[dict, dict]], measure, total: int = 0, seeds=()) -> tuple[float, float, float]:
    """Mean of measure(new) - measure(old) over `total` trials (default: the pairs), with its interval.
    `seeds`: every seed the mean is over, where the pairs are only some of the trials (the rest count as no
    change, and their seeds are resampled too)."""
    total = total or len(pairs)
    per_seed = collections.defaultdict(float, {seed: 0.0 for seed in seeds})
    for old, new in pairs:
        per_seed[new["seed"]] += measure(new) - measure(old)
    low, high = interval(per_seed, total)
    return sum(per_seed.values()) / total, low, high


def changes(pairs: list[tuple[dict, dict]], measure) -> tuple[int, int]:
    """(trials the new side has and the old has not, trials the old side has and the new has not)."""
    return (sum(measure(new) > measure(old) for old, new in pairs),
            sum(measure(new) < measure(old) for old, new in pairs))


def percent(value: float, signed: bool = False) -> str:
    return f"{100 * value:+.1f}" if signed else f"{100 * value:.1f}"


def with_interval(triple: tuple[float, float, float], scale: float = 100.0, digits: int = 1) -> str:
    mean, low, high = triple
    return f"{scale * mean:+.{digits}f} ({scale * low:+.{digits}f} to {scale * high:+.{digits}f})"


def commit(folder: Path) -> str:
    path = folder / "config.json"
    if not path.exists():
        return "?"
    config = json.loads(path.read_text())
    return f"`{config.get('commit')}`" + (" with uncommitted changes" if config.get("dirty") else "")


def report(base_dir: Path, full_dir: Path) -> str:
    base, full = keyed(load(base_dir)), keyed(load(full_dir))
    both = sorted(set(base) & set(full), key=lambda key: (key[1], key[0]))
    pairs = [(base[key], full[key]) for key in both]
    seeds = sorted({key[1] for key in both})
    text = [f"# {full_dir.name} against {base_dir.name}", "",
            f"Baseline {base_dir.name}: commit {commit(base_dir)}, {len(base)} trials. "
            f"Full {full_dir.name}: commit {commit(full_dir)}, {len(full)} trials. "
            f"Paired on {len(pairs)} trials: {len({key[0] for key in both})} objects on {len(seeds)} seeds"
            + (f" ({seeds[0]} to {seeds[-1]})." if seeds else "."), ""]
    if not pairs:
        return "\n".join(text + ["No trial is in both."]) + "\n"
    n = len(pairs)

    # 1. The whole system.
    text += ["## 1. Baseline against full", "",
             "| Measure | Baseline | Full | Difference (95 % interval) | Gained | Lost | p (sign test) |",
             "|---|---|---|---|---|---|---|"]
    for name, measure in (("Delivered, % of trials", delivered), ("Delivered with no drop, %", clean)):
        won, lost = changes(pairs, measure)
        text.append(f"| {name} | {percent(sum(measure(o) for o, _ in pairs) / n)} "
                    f"| {percent(sum(measure(f) for _, f in pairs) / n)} "
                    f"| {with_interval(difference(pairs, measure))} | {won} | {lost} | {sign_test(won, lost):.3g} |")
    text.append(f"| Points of 40, mean | {sum(points(o) for o, _ in pairs) / n:.2f} "
                f"| {sum(points(f) for _, f in pairs) / n:.2f} "
                f"| {with_interval(difference(pairs, points), 1.0, 2)} | | | |")
    lifted = lambda row: int(row.get("lift_tick") is not None)                  # noqa: E731
    won, lost = changes(pairs, lifted)
    text += [f"| Lifted, % (the pick, not changed) | {percent(sum(lifted(o) for o, _ in pairs) / n)} "
             f"| {percent(sum(lifted(f) for _, f in pairs) / n)} | {with_interval(difference(pairs, lifted))} "
             f"| {won} | {lost} | {sign_test(won, lost):.3g} |", "",
             f"First lift at the same tick in both: {sum(o.get('lift_tick') == f.get('lift_tick') for o, f in pairs)} "
             f"of {n} trials.", ""]

    # 2. By object.
    text += ["## 2. By object", "",
             "| Object | Trials | Delivered: baseline | full | gained | lost | p | No drop: baseline | full | gained "
             "| lost | p | Points: baseline | full |", "|" + "---|" * 14]
    for object_idx in sorted({key[0] for key in both}):
        group = [(o, f) for o, f in pairs if f["object"] == object_idx]
        cells = [f"{object_idx} {group[0][1].get('name')}", str(len(group))]
        for measure in (delivered, clean):
            won, lost = changes(group, measure)
            cells += [str(sum(measure(o) for o, _ in group)), str(sum(measure(f) for _, f in group)),
                      str(won), str(lost), f"{sign_test(won, lost):.2g}"]
        cells += [f"{sum(points(o) for o, _ in group) / len(group):.1f}",
                  f"{sum(points(f) for _, f in group) / len(group):.1f}"]
        text.append("| " + " | ".join(cells) + " |")
    text.append("")

    # 3. Points by rubric item.
    items = [key for key in pairs[0][1] if key.startswith("pts_")]
    if items:
        text += ["## 3. Points by item (sum over the paired trials)", "", "| Item | Baseline | Full | Difference |",
                 "|---|---|---|---|"]
        for item in items:
            old, new = sum(o.get(item, 0) for o, _ in pairs), sum(f.get(item, 0) for _, f in pairs)
            text.append(f"| {item[4:]} | {old} | {new} | {new - old:+d} |")
        text.append("")

    # 4. Each part switched off.
    ablate = full_dir / "ablate"
    if ablate.exists():
        text += ["## 4. Each part of the carry switched off", "",
                 "Full against full without the part, on the trials of the full run in which the part acted. "
                 "\"Gained by the part\" is a trial the full run delivers and the run without the part does not. "
                 "The effect is over all trials of the full run, the trials in which the part did not act "
                 "counting as no change; the check is a sample of those, run without the part.", "",
                 "| Part | Switched off by | Acted in | Delivered: gained by the part | lost by it | p "
                 "| No drop: gained | lost | p | Effect on delivered, points of % (95 %) "
                 "| Effect on no drop | Effect on points of 40 | Check: ran the same |", "|" + "---|" * 13]
        for name in COMPONENTS:
            without = keyed(load(ablate / name))
            config = json.loads((ablate / name / "config.json").read_text()) if (ablate / name / "config.json").exists() else {}
            check = {tuple(key) for key in config.get("check", [])}
            acted = [(without[key], full[key]) for key in without if key in full and key not in check]
            checked = [(without[key], full[key]) for key in without if key in full and key in check]
            same = sum(all(a.get(k) == b.get(k) for k in SAME) for a, b in checked)
            if not without:
                text.append(f"| {name} | {' '.join(COMPONENTS[name][0])} | not run | | | | | | | | | | |")
                continue
            cells = [name, " ".join(f"`{item}`" for item in COMPONENTS[name][0]),
                     f"{len(acted)} ({percent(len(acted) / len(full))} %)"]
            for measure in (delivered, clean):
                won, lost = changes(acted, measure)
                cells += [str(won), str(lost), f"{sign_test(won, lost):.2g}"]
            every = sorted({key[1] for key in full})
            cells += [with_interval(difference(acted, delivered, len(full), every)),
                      with_interval(difference(acted, clean, len(full), every)),
                      with_interval(difference(acted, points, len(full), every), 1.0, 2),
                      f"{same} of {len(checked)}"]
            text.append("| " + " | ".join(cells) + " |")
        # A trial in which a part did not act runs the same without it, so the run without the part and the full
        # run together give the outcome of every trial of the system that lacks the part.
        text += ["", f"The system with one part removed, over all {len(pairs)} paired trials:", "",
                 "| System | Delivered, % | Delivered with no drop, % | Points of 40 |", "|---|---|---|---|"]
        systems = [("baseline", [base[key] for key in both]), ("full", [full[key] for key in both])]
        for name in COMPONENTS:
            without = keyed(load(ablate / name))
            check = {tuple(key) for key in (json.loads((ablate / name / "config.json").read_text()).get("check", [])
                                            if (ablate / name / "config.json").exists() else [])}
            if without:
                systems.append((f"full without {name}",
                                [without[key] if key in without and key not in check else full[key] for key in both]))
        for name, rows in systems:
            text.append(f"| {name} | {percent(sum(delivered(r) for r in rows) / len(rows))} "
                        f"| {percent(sum(clean(r) for r in rows) / len(rows))} "
                        f"| {sum(points(r) for r in rows) / len(rows):.2f} |")
        text.append("")
        off = keyed(load(ablate / "all_off"))
        if off:
            common = [(base[key], off[key]) for key in off if key in base]
            same = sum(all(a.get(k) == b.get(k) for k in SAME) for a, b in common)
            agree = sum(delivered(a) == delivered(b) and clean(a) == clean(b) for a, b in common)
            text += [f"Every part off, on {len(common)} trials drawn at random, against the baseline's same trials: "
                     f"{same} ran the same (lift, delivery and last tick, and the failures), "
                     f"{agree} agree on delivered and on no drop. Delivered: baseline "
                     f"{sum(delivered(a) for a, _ in common)}, every part off {sum(delivered(b) for _, b in common)}.", ""]
        again = keyed(load(ablate / "repeat"))
        if again:
            common = [(full[key], again[key]) for key in again if key in full]
            same = sum(all(a.get(k) == b.get(k) for k in SAME) for a, b in common)
            text += [f"Repeats: {len(common)} trials of the full run drawn at random and run again with the same "
                     f"settings; {same} ran the same.", ""]

    # 5. Carries by travel pose and way in (full run; every trial has its carries recorded).
    def by_pose(rows: list[dict], first_only: bool) -> list[str]:
        tally = collections.defaultdict(collections.Counter)
        for row in rows:
            for carry in (row.get("carries") or [])[:1 if first_only else None]:
                lost = carry.get("lost") or ""
                cell = tally[(carry["pose"], carry["way"])]
                cell["all"] += 1
                cell["taking the pose" if "travel pose" in lost else "in the carry" if "carry" in lost else
                     "otherwise" if lost else "kept"] += 1
        lines = ["| Travel pose | Way in | Carries | Not lost | Lost taking the pose | Lost in the carry | Lost otherwise |",
                 "|" + "---|" * 7]
        lines += [f"| {pose} | {way} | {c['all']} | {c['kept']} ({percent(c['kept'] / c['all'])} %) "
                  f"| {c['taking the pose']} | {c['in the carry']} | {c['otherwise']} |"
                  for (pose, way), c in sorted(tally.items(), key=lambda item: -item[1]["all"])]
        return lines if tally else []

    for title, rows in ((f"full ({full_dir.name})", list(full.values())), (f"baseline ({base_dir.name})", list(base.values()))):
        first = by_pose(rows, True)
        if first:
            text += [f"## 5. First carry of each trial by travel pose, {title}", "", *first, "",
                     f"### Every carry, {title}", "", *by_pose(rows, False), ""]

    # 6. First carry by object and pose.
    rows = [row for row in full.values() if row.get("carries")]
    if rows:
        poses = [pose for pose, _ in collections.Counter(row["carries"][0]["pose"] for row in rows).most_common()]
        text += ["## 6. First carry by object: carries not lost / carries, by travel pose (full)", "",
                 "| Object | " + " | ".join(poses) + " |", "|" + "---|" * (len(poses) + 1)]
        for object_idx in sorted({row["object"] for row in rows}):
            group = [row for row in rows if row["object"] == object_idx]
            cells = []
            for pose in poses:
                these = [row["carries"][0] for row in group if row["carries"][0]["pose"] == pose]
                cells.append(f"{sum(not c.get('lost') for c in these)} / {len(these)}" if these else "")
            text.append(f"| {object_idx} {group[0].get('name')} | " + " | ".join(cells) + " |")
        text.append("")
    return "\n".join(text) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("base", help="benchmark of the code before the change, e.g. runs/bench_01")
    parser.add_argument("full", help="benchmark of the code with it, on the same seeds, e.g. runs/bench_02")
    args = parser.parse_args()
    full = Path(args.full).resolve()
    text = report(Path(args.base).resolve(), full)
    (full / "report.md").write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
