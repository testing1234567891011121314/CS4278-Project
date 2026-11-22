"""Benchmark 2: the same trial with and without the other ways in: the first carry's pose each way, and whether it was kept."""
import json, collections, math
from pathlib import Path
R = Path('runs')
def load(p):
    return {(r['object'], r['seed']): r for r in map(json.loads, open(p / 'results.jsonl'))}
full = load(R/'bench_02')
names = {k[0]: r['name'] for k, r in full.items()}
D = lambda r: r['delivered_tick'] is not None
C = lambda r: D(r) and r.get('drops', 0) == 0
def p_sign(a, b):
    n, k = a + b, min(a, b)
    return 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)
def first(r):
    c = (r.get('carries') or [None])[0]
    return None if c is None else (c['pose'].replace(' (low)', ''), not c.get('lost'))
for part in ('other_ways', 'joint_by_joint', 'arms'):
    cfg = json.load(open(R/'bench_02'/'ablate'/part/'config.json'))
    check = {tuple(k) for k in cfg['check']}
    wo = {k: r for k, r in load(R/'bench_02'/'ablate'/part).items() if k not in check}
    print(f'== {part}: first carry with the part -> without it; kept with / kept without / both / n')
    tr = collections.defaultdict(lambda: [0, 0, 0, 0])
    byobj = collections.defaultdict(lambda: [0, 0, 0, 0])
    for k, w in wo.items():
        a, b = first(full[k]), first(w)
        if a is None or b is None:
            tr[('no carry' if a is None else a[0], 'no carry' if b is None else b[0])][3] += 1; continue
        cell = tr[(a[0], b[0])]
        cell[0] += a[1]; cell[1] += b[1]; cell[2] += a[1] and b[1]; cell[3] += 1
        if a[0] == 'level, finger under' and b[0] == 'down, pads along':
            c = byobj[names[k[0]]]; c[0] += a[1]; c[1] += b[1]; c[2] += a[1] and b[1]; c[3] += 1
    for (a, b), c in sorted(tr.items(), key=lambda i: -i[1][3]):
        if c[3] >= 5:
            only_a, only_b = c[0] - c[2], c[1] - c[2]
            print(f'  {a:22s} -> {b:22s} n={c[3]:3d} kept with {c[0]:3d} without {c[1]:3d}; only with {only_a}, only without {only_b}, p={p_sign(only_a, only_b):.3g}')
    print('  finger under (with) against hand down (without), by object: kept with / kept without / n')
    for o, c in sorted(byobj.items(), key=lambda i: -i[1][3]):
        print(f'    {o:16s} {c[0]:3d} / {c[1]:3d} / {c[3]:3d}   only with {c[0]-c[2]}, only without {c[1]-c[2]}')
# pooled first-carry kept rate by pose (low merged), every way in
print('== first carries, full, by pose (low merged)')
t = collections.defaultdict(lambda: [0, 0])
for r in full.values():
    f = first(r)
    if f: t[f[0]][0] += f[1]; t[f[0]][1] += 1
for p, c in sorted(t.items(), key=lambda i: -i[1][1]): print(f'  {p:22s} {c[0]} / {c[1]} = {100*c[0]/c[1]:.1f} %')
print('  all first carries', sum(c[1] for c in t.values()), 'kept', sum(c[0] for c in t.values()))
