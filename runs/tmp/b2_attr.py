"""Benchmark 2: which part gives which object's change, the trials lost against the baseline, how trials end."""
import json, collections, math, statistics
from pathlib import Path
R = Path('runs')
def load(p):
    return {(r['object'], r['seed']): r for r in map(json.loads, open(p / 'results.jsonl'))}
base, full = load(R/'bench_01'), load(R/'bench_02')
D = lambda r: r['delivered_tick'] is not None
C = lambda r: D(r) and r.get('drops', 0) == 0
names = {k[0]: r['name'] for k, r in full.items()}
PARTS = ('arms', 'joint_by_joint', 'other_ways', 'weak_holds', 'reseat', 'stall')
without = {}
for name in PARTS:
    cfg = json.load(open(R/'bench_02'/'ablate'/name/'config.json'))
    check = {tuple(k) for k in cfg['check']}
    without[name] = {k: r for k, r in load(R/'bench_02'/'ablate'/name).items() if k not in check}
def p_sign(a, b):
    n, k = a + b, min(a, b)
    return 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)

print('== A. per object, per part: acted | delivered gained-lost by the part | clean gained-lost')
print('object'.ljust(18) + ''.join(p.ljust(22) for p in PARTS))
for o in range(15):
    line = names[o].ljust(18)
    for p in PARTS:
        ks = [k for k in without[p] if k[0] == o]
        dg = sum(D(full[k]) and not D(without[p][k]) for k in ks); dl = sum(D(without[p][k]) and not D(full[k]) for k in ks)
        cg = sum(C(full[k]) and not C(without[p][k]) for k in ks); cl = sum(C(without[p][k]) and not C(full[k]) for k in ks)
        line += f'{len(ks):3d} | {dg:2d}-{dl:<2d} | {cg:2d}-{cl:<2d}'.ljust(22)
    print(line)

print('\n== B. trials delivered by the baseline and not by full (33): parts that acted, and which removal restores delivery')
lost = [k for k in full if D(base[k]) and not D(full[k])]
tally = collections.Counter()
for k in sorted(lost):
    eng = full[k]['engaged']
    restores = [p for p in eng if D(without[p][k])]
    tally[tuple(restores) if restores else ('none' if eng else 'no part acted',)] += 1
    c = (full[k]['carries'] or [{}])[0]; b = (base[k].get('carries') or [{}])[0]
    print(f"  {names[k[0]]:16s} s{k[1]} acted={eng} restores={restores} first carry: {c.get('pose')} / {c.get('way')} lost={c.get('lost')} led={c.get('led_m')}")
print('  restored by:', dict(tally))
print('  base has carries recorded:', sum(bool(r.get('carries')) for r in base.values()))

print('\n== C. trials gained (99): parts acted; removal that loses delivery again')
gained = [k for k in full if D(full[k]) and not D(base[k])]
t2 = collections.Counter(); t3 = collections.Counter()
for k in gained:
    eng = full[k]['engaged']
    need = tuple(p for p in eng if not D(without[p][k]))
    t2[need] += 1; t3[names[k[0]]] += 1
for k, v in t2.most_common(): print('  ', v, 'needs', k)
print('  by object', dict(t3))

print('\n== D. how the undelivered trials end, baseline and full')
def ending(r):
    if r['lift_tick'] is None: return 'never lifted'
    if D(r): return 'delivered, no drop' if r['drops'] == 0 else 'delivered after a drop'
    if r['drops'] == 0: return 'stopped or out of ticks, no drop'
    return 'dropped, out of ticks' if r['ticks'] >= 2999 else 'dropped, attempts ran out'
for tag, rows in (('base', base), ('full', full)):
    print(' ', tag, dict(collections.Counter(ending(r) for r in rows.values())))
for tag, rows in (('base', base), ('full', full)):
    wd = [r for r in rows.values() if r['drops'] > 0]
    print(' ', tag, 'trials with a drop', len(wd), 'delivered', sum(map(D, wd)), f'{sum(map(D, wd))/len(wd):.3f}')
print('  recovery by object (full):', {names[o]: f"{sum(D(r) for r in full.values() if r['object']==o and r['drops']>0)}/{sum(1 for r in full.values() if r['object']==o and r['drops']>0)}" for o in range(15)})

print('\n== E. seeds by number delivered')
for tag, rows in (('base', base), ('full', full)):
    per = collections.Counter()
    for k, r in rows.items(): per[k[1]] += D(r)
    print(' ', tag, sorted(collections.Counter(per.values()).items(), reverse=True), 'median', statistics.median(per.values()), 'mean', statistics.mean(per.values()))

print('\n== F. points lost in full by item and object')
lostpts = collections.defaultdict(collections.Counter)
for r in full.values():
    lostpts['reach (6+6)'][r['name']] += 12 - r['pts_reach_10cm'] - r['pts_reach_in_time']
    lostpts['no drop (4)'][r['name']] += 4 - r['pts_no_drop']
    lostpts['lift by 600 (6)'][r['name']] += 6 - r['pts_lift_600']
    lostpts['lift, hold, pose (18)'][r['name']] += 18 - r['pts_lift_10cm'] - r['pts_hold_50'] - r['pts_grasp_pose']
    lostpts['all'][r['name']] += 40 - r['points']
for item, c in lostpts.items():
    print(' ', item, sum(c.values()), [(n, v) for n, v in c.most_common(8) if v])

print('\n== G. clocks')
for tag, rows in (('base', base), ('full', full)):
    print(' ', tag, 'core-hours', round(sum(r['wall_s'] for r in rows.values()) / 3600, 1))
abl = 0; n = 0
for d in (R/'bench_02'/'ablate').iterdir():
    rows = load(d); abl += sum(r['wall_s'] for r in rows.values()); n += len(rows)
print('  ablation trials', n, 'core-hours', round(abl / 3600, 1))
dt = [r['delivered_tick'] for r in full.values() if C(r)]
print('  clean delivery tick median', statistics.median(dt), 'weak-hold first carries delivered tick median',
      statistics.median([r['delivered_tick'] for r in full.values() if C(r) and r['carries'] and r['carries'][0]['weak']]))
print('  first carry step mm median (full)', statistics.median([r['carries'][0]['step_mm'] for r in full.values() if r['carries'] and r['carries'][0].get('step_mm')]))
print('  weak first carries by object', dict(collections.Counter(r['name'] for r in full.values() if r['carries'] and r['carries'][0]['weak'])))
