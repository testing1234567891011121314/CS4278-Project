"""Cross-check runs/bench_02/report.md from the raw rows, and attribute changes to parts."""
import json, collections, sys
from pathlib import Path
R = Path('runs')
def load(p):
    return {(r['object'], r['seed']): r for r in map(json.loads, open(p / 'results.jsonl'))}
base, full = load(R/'bench_01'), load(R/'bench_02')
D = lambda r: r['delivered_tick'] is not None
C = lambda r: D(r) and r.get('drops', 0) == 0
names = {k[0]: r['name'] for k, r in full.items()}
print('trials', len(base), len(full), 'same keys', set(base) == set(full))
for nm, f in (('delivered', D), ('clean', C)):
    print(nm, sum(map(f, base.values())), sum(map(f, full.values())),
          'gained', sum(f(full[k]) and not f(base[k]) for k in full), 'lost', sum(f(base[k]) and not f(full[k]) for k in full))
print('lifted', sum(r['lift_tick'] is not None for r in base.values()), sum(r['lift_tick'] is not None for r in full.values()))
print('points', sum(r['points'] for r in base.values()), sum(r['points'] for r in full.values()))
for tag, rows in (('base', base), ('full', full)):
    reach = sum(12 - r['pts_reach_10cm'] - r['pts_reach_in_time'] for r in rows.values())
    nod = sum(4 - r['pts_no_drop'] for r in rows.values())
    tot = sum(40 - r['points'] for r in rows.values())
    print(tag, 'points lost', tot, 'reach', reach, 'no_drop', nod, 'after lift share', round((reach + nod) / tot, 4))
# parts
parts = {}
for name in ('arms', 'joint_by_joint', 'other_ways', 'weak_holds', 'reseat', 'stall'):
    cfg = json.load(open(R/'bench_02'/'ablate'/name/'config.json'))
    check = {tuple(k) for k in cfg['check']}
    rows = load(R/'bench_02'/'ablate'/name)
    acted = {k: r for k, r in rows.items() if k not in check}
    eng = {k for k, r in full.items() if name in r['engaged']}
    parts[name] = acted
    print(name, 'acted rows', len(acted), 'engaged in full', len(eng), 'equal', set(acted) == eng, 'check', len(check),
          'check disjoint from engaged', not (check & eng))
json.dump({n: [list(k) for k in a] for n, a in parts.items()}, open('runs/tmp/b2_acted.json', 'w'))
# engaged overlaps
cnt = collections.Counter(tuple(sorted(r['engaged'])) for r in full.values())
print('engaged combinations:'); [print('  ', v, k) for k, v in cnt.most_common()]
print('no part acted:', cnt[()], ' any part:', len(full) - cnt[()])
