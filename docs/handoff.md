# Handoff: solver rewrite and the report, 2026-11-22 (submission day)

Read this first, then **`NAVIGATE_V2.md`**: the full record of the carry
(findings, problems, hypotheses with verdicts, every table, sections 7.1 to
7.26, the change log V2-1 to V2-31). `docs/literature_review.md` is the
literature. `GRASP.md` is the pick, which is frozen. `NAVIGATE.md` is the
first carry, measured on a regressed grasp and a shorter maze.
`docs/plan.md` is out of date.

## Where things stand

Branch `rewrite/solver`, pushed; one commit of documents after `4b98d4b`
(the user's push of benchmark 2).

| Commit | What it is |
|---|---|
| `b701e73` | The pick the user verified. Still the reference for the grasp. |
| `dd0ab31` | The carry rebuilt on top of the untouched pick (V2-1 to V2-19). |
| `376e759` | `tools/bench.py`. **The code of benchmark 1.** |
| `1bf6206` | The user's benchmark 1 results, `runs/bench_01`. |
| `fa6a3cf` | Documents: benchmark 1 analysed, V2-20 to V2-31 logged, the literature review. |
| `4e155e7` | The carry V2-20 to V2-31, `tools/bench.py --ablate`, `tools/bench_report.py`, `tools/trace_trial.py`. **The code of benchmark 2.** |
| `4b98d4b` | The user's benchmark 2 results, `runs/bench_02` (the full run and `ablate/`). |

The commit after `4b98d4b` (the one that carries this file) holds: this
file; section 7.26 and two table rows in `NAVIGATE_V2.md`;
`runs/bench_02/report.md` and the three scripts `runs/tmp/b2_*.py` that
section 7.26 cites (git-ignored, added with `git add -f`); one corrected
author initial in `docs/lit_transport_holds.md` (H. Pham, not R. Pham).
Commit and push only when the user asks.

`assets/maze.urdf` and `pnr/pyvenv.cfg` show as modified and are not ours:
leave them out of every commit. The untracked `old/`, `pnr/bin`, `pnr/lib`,
`pnr/lib64` and `pristine/` are not ours either.

Nothing is running, here or on the Xeon.

## Benchmark 2 is back and written up (2026-11-22)

`runs/bench_02`: commit `4e155e7`, not dirty, the same 1,500 trials as
benchmark 1, and 1,666 trials of ablation. Done with it:

- `runs/bench_02/report.md` made (`tools/bench_report.py runs/bench_01
  runs/bench_02`) and its counts recounted from the rows
  (`runs/tmp/b2_check.py`; per-object attribution `runs/tmp/b2_attr.py`;
  the same trial in two poses `runs/tmp/b2_pose_pairs.py`).
- **The three controls pass** (240 of 240, 100 of 100, 40 of 40).
- **No default was changed.** No part hurts when removed alone. Joint by
  joint stays on: 29 deliveries gained and 16 lost in the 323 trials in
  which it acts (the ten local trials of section 7.25 pointed the other
  way).
- `NAVIGATE_V2.md` section 7.26 and the task table hold all of it.
- The paper's Results (five subsections) and abstract are filled from it.

The user's standard for this, in their words: "this is not a cheap test"
and "the results when back must be clearly explained and ablated, ready
for neurips reporting".

## The paper

A Claude doc, the user's submission in the style of a NeurIPS paper:
https://claude.ai/artifact/XGYA8M1uTLg5zabGCx1pio ("Pick-Nav-Reach:
Classical Grasping, Navigation, and Delivery"). The user said it was
converted from their LaTeX and paraphrased by another model, so the LaTeX
source is theirs and edits made in the doc may have to be carried back
(my reading; ask). Read and edit it with the docs tools, never by fetching
the link.

What I did to it on 2026-11-22, at the user's request:

- Reviewed it; 13 corrections laid as suggestions, then all accepted on
  the user's word.
- **Emptied the Results section** (heading kept): it described the old
  solver (5 of 10 on five objects).
- Rewrote Future Work for the current solver, with no result figures.
- Added to the Methodology "Choosing the travel pose" and "Evaluation
  protocol" (with a table of the six ablated parts); added Limitations and
  References (16 entries) and 18 citations in the text.

Done in it on 2026-11-22, after benchmark 2:

- **Results** filled, five subsections: baseline against full, by object,
  ablation (the three controls first), carries by travel pose, what is
  still lost. Written into the doc directly, not as suggestions.
- **The abstract** now quotes benchmark 2 (81 % delivered, 69 % with no
  drop, +4.4 and +6.5 points against the baseline, 68 % of lost points
  after the lift).
- **An appendix, "Appendix: Algorithms"**, after the References: 23
  algorithms (A.1 to A.23) in the order a trial meets them, each written
  from the code as it stands at `4e155e7`, with its default parameters. A
  change to a default in `solver.py`'s `Config` or the settings classes
  has to be carried into it by hand.
- **One suggestion is open**, for the user to accept or reject: Future
  Work said small objects are lost while the pose is taken "rather than
  during the drive"; benchmark 2 says 23 of the mug's 49 lost first
  carries and 14 of the strawberry's 62.

Still to do in it:

- **"The median carry speed of 12.8 mm per tick"** in "PyBullet quirks" is
  benchmark 1's figure over trials with a log. Over every first carry of
  benchmark 2 the median is 12.2. Not changed: the 202 m/s² beside it is
  derived from 12.8.
- **The tables may have to be carried back to the user's LaTeX.**
- **References to glance at**: [12] and [14] could not be re-fetched;
  [13], [15] rest on abstracts; the venues of [8] (ICRA 2019) and [11]
  (IROS 2021) are from memory. Nguyen [3] was verified from abstracts only.
- The jolt figures in "PyBullet quirks" (202 m/s² and so on) are derived
  from the documented control law by an agent and have not been measured
  in a trace. The paper says so.

## Results to quote, with what they rest on

**Benchmark 1** (`runs/bench_01`, the Xeon, commit `376e759`: the carry
before V2-20). 15 objects on seeds 1000 to 1099; the seed is the layout,
the env and the cloud. 3,000 ticks, one trial each.

| | Of 1,500 trials |
|---|---|
| Lifted | 1,426 (95 %) |
| Delivered | 1,142 (76 %) |
| Delivered with no drop | 939 (63 %) |
| Mean points of 40 | 34.2 |

Delivered per object: potted meat and foam brick 100, mustard 98, banana
96, soup can 94, bowl 87, sugar box, apple, mug and hammer 84, strawberry
77, bleach 53, marker 47, can 30, plate 24. 72 % of the points lost are
lost after the lift. It cost 56 core-hours (3.7 h at `-j 15`).

**Benchmark 2** (`runs/bench_02`, the Xeon, commit `4e155e7`: V2-31), the
same 1,500 trials, paired with benchmark 1 (section 7.26):

| | Benchmark 1 | Benchmark 2 | Difference (95 %) | Gained | Lost |
|---|---|---|---|---|---|
| Lifted | 1,426 | 1,426 | the same tick in 1,500 of 1,500 | | |
| Delivered | 1,142 (76.1 %) | 1,208 (80.5 %) | +4.4 points (+3.2 to +5.7) | 99 | 33 |
| Delivered with no drop | 939 (62.6 %) | 1,036 (69.1 %) | +6.5 points (+5.0 to +8.0) | 123 | 26 |
| Mean points of 40 | 34.15 | 34.88 | +0.72 (+0.55 to +0.90) | | |

- By object: bleach 53 to 77, sugar box 84 to 98 (no drop 53 to 88),
  hammer no drop 44 to 60, can 30 to 38. **The soup can is worse, 94 to
  90**: finger-under carries that the new parts made possible; hand down
  never loses it (31 of 31).
- Ablation, effect on delivered and on no drop, in points: both other ways
  in +2.4 and +4.4 (joint by joint alone +0.9 and +2.2), stall +0.7 and
  +0.6, the weak hold's pace +0.5 and +0.8, more arms +0.3 and +0.8,
  re-seating 0 and 0.
- The weak hold's pace gains 17 bleach deliveries and loses 5; for the can
  it gains 2 and loses 6.
- The same trial in two poses (the ablation of the ways in): finger under
  keeps the object in 219 of 274 first carries, hand down in 193; level in
  73 of 77, hand down in 40.
- All 15 delivered on 4 seeds of 100 (1 before); the middle seed 12.

**Old against new on four local draws** (all 15, one trial each; old is
`376e759`, new is `4e155e7`; sections 7.22 and 7.25):

| Draw (layout, cloud seed) | Delivered, old | new | No drop, old | new |
|---|---|---|---|---|
| 42, 2 | 11 | 13 | 9 | 11 |
| 42, 3 | 12 | 13 | 10 | 11 |
| 67, 67 | 11 | 11 | 9 | 10 |
| 21, 21 | 13 | 12 | 11 | 10 |

The trial lost on 21 is the bowl, after finger under was reached joint by
joint; with `travel_more_ways=1` it is delivered. Layout 42 is the layout
the changes were tuned on. All lift ticks equal the pick alone. The 105
fast tests pass.

## What benchmark 1 showed (section 7.24)

- **The ticks leave room for one carry.** The pick ends near tick 290, the
  route is 22 m (17.6 to 30.1 over 100 layouts), so the first carry runs
  near 13 mm per tick and arrives near tick 2,460.
- **A drop is recovered in 47 % of trials**, by object from 74 % (mug) to
  3 % (can).
- **The layout is not a shared cause.** Failures are close to independent
  between the objects of a seed. Route length matters a little.
- **Within one object, a late pick does not make a drop likelier.** The
  pooled trend is the object's identity.
- **The solver's friction limit does not predict a drop.**
- **Two kinds of loss.** Heavy or weakly held objects go in the drive, at
  any distance. Small ones (strawberry, mug) go while the hand is moved
  into its pose, or in the first half metre.
- **Not readable from it:** whether hand down loses more than finger
  under, and whether slower is safer. Only failed trials kept a log.
  Benchmark 2 records every carry of every trial for this.
- **The pick:** marker lifted by tick 600 in 17 trials of 100; never
  lifted: marker 34, can 22, plate 18. Five objects never lift on their
  first attempt (mustard, bleach, mug, plate, marker).

## The user's rules for this work

- **The grasp library is frozen.** Nothing under
  `custom_libraries/grasping/` is edited, nor the pick's functions in
  `solver.py` (`_settle` to `_drag`). Grasp and travel pose are two
  problems: never change the grasp to fix a carry. The limits may be
  loosened for a late pick once the carry is settled; the grasp is not to
  be rewritten.
- **Planning limits count work, not seconds.**
- **Every change gets old against new on the same seeded draws**, with a
  baseline. Outcomes flip with the cloud.
- **A benchmark on the Xeon is costly**: design it to run once, try the
  whole pipeline here first, and deliver the result explained and ablated.
- **`NAVIGATE_V2.md` holds all findings, problems, hypotheses and fixes**,
  each with its seed, maze and number of trials.
- **A pose per object** only if proven to reach 15 of 15.
- **Work in the main session**; agents only when the user asks, with short
  briefs. No side branches or worktrees. Every file inside the repo
  (`runs/tmp/` for scratch).
- **Edit with the Edit and Write tools**, never `sed` or patch scripts.
- **One simulation at a time**, `-j 1`, under the memory cap (`$LOCK`
  below). Never edit code while a run is in progress.
- **No `rm -rf`.** New names for run output; list and tell before deleting.
- **Commit and push only when asked.**
- **Do not open the videos or logs in the Xeon's zips** under `runs/`
  (about 1 GB each); `runs/xeon/NAME/` holds the small files.
- Scoring: `readme.md`, section Rubrics. Five test objects, five test
  mazes, unprovided seeds; the layout may differ by examiner.

## How the carry works now

`custom_libraries/navigation/transport.py` plans everything after the pick;
`solver.py`'s `_back_off`, `_navigate`, `_reach` and `_retry` call it.

1. **Back-off** (`choose_back_off`): a held object that rests against
   something is first taken straight off it. Then the shortest retreat,
   none if that will do, from whose end the travel pose can be taken and
   the carry can start; judged a second time with the other way in before
   the fixed 0.6 m retreat.
2. **Travel pose** (`plan_travel`, `travel_poses`): the first pose in the
   rule's order that is within reach, has a clear way in, and in which the
   whole carry is clear with a 12 mm margin. The rule: a hand that does
   not point down is kept and folded; from 0.6 kg, level first; otherwise
   one finger under the object, then the hand straight down, then folded.
   Up to three arms are tried for a pose. Ways in: straight; then the move
   before the turn; then joint by joint (never for a weak hold). The object
   rides 0.60 m from the base's centre, at the goal's height where the arm
   can.
3. **Carry** (`plan_carry`, `navigation/carry.py`): the object is led along
   the route, the base an arm's length behind; the drive at 1.25 times
   what the ticks left require, or exactly that for a weak hold (the
   solver's own limit under 4.5 mm per tick). An object that has sunk
   35 mm across the pads of a level hand is re-seated: the drive rests and
   the wrist rolls half a turn.
4. **Reach** (`reach_step`, `plan_raise`, `plan_reach`): the base and torso
   close the gap with the arm still.
5. **Retry**: a carry refused with the object in the hand is planned again
   with a smaller margin and the pose chosen again. After a drop the hand
   opens where it is and the pick starts again from where the object lies.

Every part added since benchmark 1 has a setting that switches it off:
`travel_arms=1`, `travel_more_ways=0` (or 1 for joint by joint alone),
`carry_fragile=0`, `reseat_sag=0`, `free_reach=0`,
`carry_retry_margins=(1.0,)`. With all of them off the code reproduced
`376e759` tick for tick on the two local control trials.

## What is known about the carry (the evidence is in `NAVIGATE_V2.md`)

- **The base jolts the hand at every tick**, by an amount set by the step
  alone (derived, not measured: about 21 g at 12.8 mm per tick). Friction
  cannot hold the heavy objects through it, so a friction-held object
  slips a little every tick. What decides a carry is where the slips add
  up to and what stops them.
- **Each pose blocks two directions of slip and leaves one to friction.**
  Hand down: the object slides out along the fingers. Hand level: into the
  palm, where it stops, then it sinks across the pads, but only while the
  base turns through bends. Finger under: the weight rests on a finger.
- **A pose reached joint by joint is a poor start for a carry.**
- **The level poses are wide.** Finger under is refused on some routes by
  a real collision of the upper arm, not by the margin (banana, layout 67).
- **Outcomes are repeatable and very sensitive.** A degree or two changes
  a trial. Single trials do not separate close alternatives.

## Open, in the order I would take them

1. Benchmark 2 and the paper's Results: done (above).
2. **Which pose loses what**: answered in section 7.26. Hand down is the
   worse pose for the sugar box and the hammer, no different for the
   strawberry, and the better one for the soup can and perhaps the bowl.
   A rule that puts hand down first for those two is a pose per object,
   which the user allows only if proven to reach 15 of 15; not done.
3. **Small objects lost on the way into the pose** (strawberry, mug):
   check that a finger or the palm stays under the object along the whole
   way in.
4. **Recovery after a drop** for the can, bleach and plate: the pick from
   the maze floor finds no plan or no parking pose.
5. **The pick of the marker, can and plate**, if the user loosens the
   limits: the can's collision hull is about a millimetre wider than the
   open hand (an agent's measurement, not re-checked); the plate's rim
   grasp is not force-closure.
6. **The unseeded cloud.** Every benchmark trial is seeded; the graded
   runs are not. The four Xeon runs of 2026-11-21 (section 7.13: 11, 12,
   12, 12 of 15) are the only unseeded data.
7. Ideas from the literature not tried (`docs/literature_review.md`,
   section 7): banking the hand in bends; a clearance cost in the route;
   creep per metre against speed measured in the planning copy.

## How to run things

Always from the repo root, one simulation at a time:

    LOCK="flock /home/shared/pnr_sim.lock systemd-run --user --scope --quiet -p MemoryMax=2000M -p MemorySwapMax=0"

    # the whole trial, all 15, run.sh's maze, one cloud seed (about 25 minutes: run it in the background)
    $LOCK pnr/bin/python tools/pick_all.py --batch-maze --cloud-seed 2 --deliver --out runs/NAME > runs/NAME.log 2>&1

    # another layout, as the benchmark does (layout, env and cloud seeded alike)
    $LOCK pnr/bin/python tools/pick_all.py --batch-maze --maze-seed 67 --cloud-seed 67 --deliver --out runs/NAME > runs/NAME.log 2>&1

    # some objects only, with the solver's record of each attempt
    $LOCK pnr/bin/python tools/pick_all.py --batch-maze --cloud-seed 2 --deliver --objects 8 11 -v --out runs/NAME

    # one line per trial of a run: lift, delivery, each attempt's pose, speed and loss
    pnr/bin/python runs/tmp/show_runs.py runs/NAME

    # a setting for one run
    ... --set travel_more_ways=1 carry_haste=1.5

    # the pick alone (the regression check for the pick: lift ticks must equal runs/pick_cycles/seedN)
    $LOCK pnr/bin/python tools/pick_all.py --batch-maze --cloud-seed 2 --out runs/NAME

    # a whole trial with a line per tick of the carry: where the object sits in the hand, the forces
    $LOCK pnr/bin/python tools/trace_trial.py 0 --cloud-seed 5 > runs/NAME.log

    # the benchmark over many seeds: headless, no video, trials in parallel (for the Xeon; here -j 1 under $LOCK)
    pnr/bin/python tools/bench.py runs/NAME --seed-range 1000 1100 -j 30 --ablate
    pnr/bin/python tools/bench.py runs/NAME --seed-range 1000 1100 -j 30 --ablate --resume    # after an interruption
    pnr/bin/python tools/bench_report.py runs/BASELINE runs/NAME                              # writes runs/NAME/report.md

    # the old code, for a baseline: a copy of 376e759's code under runs/tmp/base_376e759 (its output lands in that folder's runs/)
    $LOCK pnr/bin/python runs/tmp/base_376e759/tools/pick_all.py --batch-maze --maze-seed 67 --cloud-seed 67 --deliver --out runs/NAME

    # fast tests (37 simulator tests are skipped unless PNR_SIM_TESTS=1)
    pnr/bin/python -m unittest discover -s tests

Where to look in a full run's log (`runs/NAME/NN_object.log`): the line
`transitions:` gives the tick of every state change; each `attempt:` line
has `back_off`, `travel` (the pose, the arm, the way in, and why earlier
poses were refused), `carry` (route length, speed), `fragile`, `reseats`,
and, after a drop, `lost` (tick, phase, metres led).

## What is lying around under `runs/` (git-ignored unless added by hand)

- `runs/bench_01/`: benchmark 1, committed.
- `runs/deliver_new_m67`, `runs/deliver_new_m21`, `runs/ways1_*`,
  `runs/probe_m67_banana_margin5`, `runs/deliver_20_s2`,
  `runs/deliver_21_s3`: the local runs sections 7.22 and 7.25 cite.
- `runs/tmp/base_376e759/`: the old code's copy, with its own `runs/`
  (`deliver_old_m67`, `deliver_old_m21`). Not a git worktree.
- `runs/tmp/smoke_base`, `runs/tmp/smoke_full`: the rehearsal of the
  benchmark pipeline on four trials. `runs/tmp/report_selftest`: made-up
  data for the report script. All three can go once someone has looked.
- `runs/tmp/reapply/`: three stale file copies from 2026-11-21.
- `runs/xeon/NAME/`: the small files of the user's four unseeded runs.

## Pitfalls

- Do not edit code or run tests while a background run is in progress: each
  trial runs in a fresh process that imports whatever is on disk. This
  includes `tools/bench.py` itself while a benchmark runs.
- A full run takes longer than the 10 minutes a foreground command gets:
  start it in the background, and do not poll it.
- `pgrep -f PATTERN` matches the shell that runs it. Use a bracket:
  `pgrep -f "tools/pick_al[l]\\.py"`.
- A shell `cd` persists between commands: use absolute paths or `cd` back.
- `runs/` is git-ignored; result tables cited in the documents were added
  with `git add -f`.
- The system `python3` has no numpy; use `pnr/bin/python`. `unzip` is not
  installed; use Python's `zipfile`.
- `rest_z` in a result row is the object's own origin: the mug (0.045) and
  the marker (0.011) lie on the floor at the start, in every run.
- In the paper's doc, suggestions are accepted one call each (a batch of
  accepts is refused), and whole-block deletes need the hash and the
  revision of the read that showed them.

## Things I got wrong, so the next session does not repeat them

- **I read causes off the failures' logs.** "A late pick causes drops",
  "hand down loses the object" and "slower is safer" all came from trials
  that had failed; the trials that succeeded had no log. The user asked
  whether there were confounding factors, and three of the claims did not
  survive. Ask what the comparison group is before believing a pattern.
- **I told the user 26 of 30 outcomes were identical; it was 28.** Count
  from the table before writing the number.
- **I tuned on layout 42 and reported its gain first.** On two other
  layouts the gain was none. Say which layout a change was tuned on.
- **An agent's review gave a wrong author initial** (R. Pham for H. Pham),
  found only when the citation was checked against the page. Agents'
  derivations and measurements in `docs/lit_*.md` are not checked by a
  second reading; check before quoting.
- **I patched files with shell scripts** on 2026-11-21 and the user saw no
  diffs. Edits go through the Edit tool.
- **I guessed the sugar box touched the table.** The contact record said
  no. Trace first.

## A word to whoever picks this up

The question changed twice. It was "how does the object stay in the
hand", and much of that was the carry throwing it. Then it was "which
pose holds", and benchmark 1 showed that most of what I thought I knew
about that came from looking only at failures. What is solid now is
narrower and better founded: the pick works for twelve objects, the robot
arrives when the object stays in the hand, and the object leaves the hand
through small slips at every tick that add up in a direction the pose
decides.

Benchmark 2 is the first measurement built to answer that properly. Read
its controls before its headline, and when a part turns out to hurt, say
so plainly in the paper: an ablation with a negative row is more
convincing than one without.

The user has been running the expensive machine by hand through the
night, on the day the work is due. Have the next thing ready before they
ask, and do not spend their run on something that could have been tried
here first.
