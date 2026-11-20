# Handoff: solver rewrite, end of the session of 2026-11-20

Read this first, then `docs/plan.md` (the section "Segment 3 from here"),
then `NAVIGATE.md` and `GRASP.md` for the record of what was tried.

## Where things stand

Branch `rewrite/solver`, pushed up to `69c796e`.

| Commit | What it is |
|---|---|
| `b701e73` | The pick: go to object, grasp, lift, hold, back off. **The user's reference for the grasp.** No carry. |
| `947a0e8` | Same solver; `run.sh`, `run_batch.py`, `trial_worker.py` made to work with it. |
| `f09377d` | First carry (side hold, arm fold). 11 of 15 delivered on one seed. It also changed which grasp is chosen, which broke the plate's pick under `run.sh`. |
| `39b169f` | Grasp behaviour restored behind switches; planning limits counted in work. Superseded. |
| `69c796e` | **Head.** Clean revert: `solver.py` and every pick-path file have the same git blob hash as in `b701e73` (`planning_sim.py` as in `947a0e8`). The solver has no carry: after the back-off it holds its pose. |

**Uncommitted in the working tree: the port from time limits to cycles**
(`common.py`, `grasping/ik.py`, `grasping/arm_planner.py`,
`navigation/parking.py`, `navigation/reach.py`, three settings in `solver.py`,
`tests/test_ik.py`, `tools/experiments/reach_probe.py`,
`navigation/travel_pose.py`). It is the only change on top of the clean
revert. `Deadline(seconds)` became `Budget(steps)`; the settings are
`ready_work` 6000, `parking_work` 40000, `planning_work` 40000,
`PlannerSettings.tree_work` 12000 with `extend_cost` 4,
`ParkingSettings.pose_cost` 20. The 105 fast tests pass with it.

`assets/maze.urdf` and `pnr/pyvenv.cfg` show as modified and are not ours:
leave them out of every commit.

## What was running when the session ended

A regression run of the pick with the cycles port: all 15 objects, cloud
seeds 1 (twice), 2, 3, 4, 5, in `run.sh`'s maze. About 75 minutes.

    logs     runs/pick_cycles_seed1_a.log, _seed1_b.log, _seed2.log ... _seed5.log
    results  runs/pick_cycles/seed1_a/results.csv ... seed5/results.csv

If it did not finish, rerun it (command under "How to run things"). Then,
before committing the cycles port, check in this order:

1. **Repeatable:** `seed1_a` and `seed1_b` identical, object by object.
2. **No regression:** `seed1_a` against `runs/pick_eq_old_1.log` (the code of
   `b701e73`, same seed and maze). Expect the mug one attempt sooner. The
   plate is the known difference (see below).
3. **The pass bar:** per object, how many of the five seeds are lifted 10 cm
   and held 50 ticks by tick 600. The bar is 4 of 5.

Commit the port only if 1 and 2 hold. Push only when the user asks.

## The user's rules for this work

These were given during the session and override my own preferences.

- **Grasp and travel pose are two problems.** "An optimal grasp is not an
  optimal travel pose." Never change which grasp is chosen to fix a loss in
  the carry.
- **The grasp is `b701e73`'s, byte for byte.** Not the same behaviour behind
  switches: the same file hashes. Anything else is added on top as its own
  change.
- **Planning limits count cycles of work, never time.**
- **Always test for regressions:** same cloud seeds, `run.sh`'s maze, old
  code against new, nothing allowed to get worse.
- **Empirical only.** "Ignore all preconceptions of optimal grasp" (the
  user's own included). No hold is kept because it sounds right.
- **Yaw travel.** The base turns with the route so the gripper stays lined up
  with the direction of travel. Do not slide the base sideways with a fixed
  heading (proposed and rejected).
- **The back-off must not be hard-coded.** After a pick from the floor,
  backing straight away has caused collisions. Choose it from the scene.
- **Order of work:** (1) measure whether the pick reaches 4 of 5 per object;
  (2) find the best travel pose; (3) only then return to the grasp.
- **Target:** 15 of 15 delivered; a re-pick only where there is no choice.
  The user expects "a clear optimal travel mechanism that transports all
  objects once grasped".
- Standing rules from before: work in the main session; no side branches or
  worktrees; every file inside the repo; commit and push only when asked; one
  simulation at a time under the memory cap; nothing copied from `old/`.

## Things I got wrong, so the next session does not repeat them

- **"15 of 15" for the pick was two cloud draws** (seeds 300 and 301, the
  env's own maze). I went on quoting it as if it described the code. On
  seed 1 in `run.sh`'s maze `b701e73` lifts and holds 13 of 15. Quote every
  result with its seeds, maze and number of trials.
- **All my carry evidence was one cloud seed in one maze that is not
  `run.sh`'s.** `run.sh` uses an unseeded cloud and the maze `run_batch.py`
  builds (layout seed 42), not the env's own (layout seed 5).
- **I changed grasp selection to help the carry** (centre-of-mass ranking, a
  grip under the hammer's head). That is what the user saw as the plate no
  longer being grasped.
- **"Every hand pointing straight down is delivered" was false.** Over 72
  carries: straight down 31 delivered, 21 lost; tilted 4 and 6.
- **Time-limited planning made runs unrepeatable**, which I found late. See
  the plate, below.

## The plate on seed 1 (unresolved)

Four runs of the plate's pick, cloud seed 1, `run.sh`'s maze:

| Run | Result |
|---|---|
| `b701e73`'s code, as part of all 15 | lifted at tick 349 |
| `b701e73`'s code, plate alone | not lifted, 6 attempts |
| grasp restored, time limits | not lifted, identical to the row above |
| grasp restored, work limits | not lifted, identical |

All four are the same up to tick 199 (settle, drive, rake 45 cm to the
table's edge). The run that succeeds starts its second attempt at tick 251,
the others at 255, and all choose the same rim grasp. In the failing runs the
second attempt ends "arm did not reach the end of the approach". The
likeliest cause of the 4-tick difference is the old 3-second limit cutting
short the arm's return to its ready pose (the ready-pose solve, or the path
to it); this was not traced. With cycle limits the plate fails on this seed
every time. The real weakness is that approach after the rake. It belongs to
step 3 (the grasp), not before.

## What is known about the carry

From `NAVIGATE.md` (eleven full attempts, the traces, and the library run):

- Finding the route and driving it has never failed. Every loss is the
  object leaving the hand, a route refused because the arm was too wide, or
  the ticks running out.
- The env's base is position-controlled: its speed jumps at every tick, by
  about 16 g for one physics step at 10 mm per tick. The route is 21 m and
  about 2,400 ticks are left after the pick, so 9-12 mm per tick is needed.
- The line between the pads is a loose hinge (no spinning friction).
- Library run, 63 further objects, committed carry of `f09377d`
  (`runs/library_01/report.txt`): **three losses in five happen in the first
  400 ticks after the back-off** (re-pose, torso rise, wrist rolls, turn on
  the spot, first metre). Speed is the clearest predictor (10-12 mm per tick
  13 of 37 delivered; over 22 mm per tick 2 of 17). Nothing over 0.6 kg was
  delivered. Grasps wider than 85 mm deliver 2 of 15. Contacts near the back
  of the pads deliver 17 of 45, further out 7 of 44. Hand direction, wrist
  steering and the size of the first turn do not separate delivered from
  lost.
- Tried and not kept: fingers fully closed for the carry (9 of 15 against
  11); half the squeeze; standing a tilted hand upright; the torso rising
  during the whole drive (cost the strawberry and hammer).
- Heavy objects creep down the fingers about 2 mm per metre whatever the
  squeeze.

## Next steps, in the user's order

1. **Finish the pick measurement** (the run above) and report per object out
   of five. Commit the cycles port if it passes its two checks.
2. **Run the travel-pose survey.** `tools/travel_poses.py` is written and
   compiles but has **never been run**: expect first-run bugs. It picks each
   object once with the solver, saves the simulator state at the end of the
   back-off, and replays 13 poses from that state, each with the same turn
   on the spot and 2.5 m of real route. It records which step loses the
   object, slip per metre and turn in the hand. Poses are in
   `custom_libraries/navigation/travel_pose.py` (`SURVEY`): as lifted; folded
   at 0.70 and 0.40 m; pads' line along the base's heading at 0.70 and 0.40;
   pads sideways; hand stood down with pads along or across; level; 45 up;
   three with one finger under the object. Run it at more than one speed
   (`--step-mm`).
3. **Build the carry from the survey's table**, as its own module on top of
   the untouched pick (a subclass of `Solver`, or handlers the solver calls),
   not by editing the pick's functions. The old carry code is in `f09377d`
   (`solver.py`: `_navigate`, `_plan_carry`, `_plan_hold`, `_reach`).
   `navigation/carry.py` (route planning, `CarryFollower`) is still in the
   tree and tested by `tests/test_carry.py`. Include a back-off chosen from
   the scene.
4. **Then the grasp**, for objects under 4 of 5: the plate's approach after
   the rake; the marker (thrown out of the table, landing chaotic); the
   hammer (lifted, not held 50 ticks on seed 1); "object moved during the
   drive" for round or disturbed objects.

Open questions never answered by the user: the primary measure; which five
trials per object count; the submission date.

## How to run things

Always from the repo root, one simulation at a time:

    LOCK="flock /home/shared/pnr_sim.lock systemd-run --user --scope --quiet -p MemoryMax=2000M -p MemorySwapMax=0"

    # the pick, all 15, run.sh's maze, one cloud seed (about 12 minutes)
    $LOCK pnr/bin/python tools/pick_all.py --batch-maze --cloud-seed 1 --out runs/NAME

    # the regression run that was in progress
    for run in 1_a 1_b 2 3 4 5; do seed=${run%%_*}; $LOCK pnr/bin/python tools/pick_all.py --batch-maze \
        --cloud-seed $seed --out runs/pick_cycles/seed$run > runs/pick_cycles_seed$run.log 2>&1; done

    # the same with b701e73's code, exported to runs/tmp/pre_nav (not a worktree; plain files)
    (cd runs/tmp/pre_nav && $LOCK ../../../pnr/bin/python tools/pick_all.py --batch-maze --cloud-seed 1 --out runs/NAME)

    # the travel-pose survey
    $LOCK pnr/bin/python tools/travel_poses.py --cloud-seed 1 --out runs/travel_01

    # where a --deliver run loses its objects
    pnr/bin/python tools/carry_report.py runs/library_01

    # fast tests (37 simulator tests are skipped unless PNR_SIM_TESTS=1)
    pnr/bin/python -m unittest discover -s tests

`runs/tmp/pre_nav/` holds `b701e73`'s solver and libraries with the current
`tools/trial.py` and `tools/pick_all.py` copied in and `assets` linked. If it
is gone, recreate it with `git archive 947a0e8 solver.py custom_libraries
run_batch.py trial_worker.py pick_nav_reach_env_multi.py utils.py
maze_utils.py | tar -x -C runs/tmp/pre_nav`, copy the two tools in, and link
`assets`.

The library: `assets/ycb_urdfs` is a git submodule (78 objects);
`pnr/bin/python tools/ycb_library.py` regenerates the 63 the course lacks
into `assets/ycb_extra/` (git-ignored); `tools/pick_all.py --library
assets/ycb_extra` runs them. It is a second test set, not something to tune on.

## Pitfalls

- Do not edit code or run tests while a background run is in progress: each
  object runs in a fresh process that imports whatever is on disk.
- `pkill -f PATTERN` and `pgrep -f PATTERN` match the shell that runs them
  and kill it (exit 144). Use a bracket in the pattern: `pgrep -f
  "tools/pick_al[l]\\.py"`.
- No `rm -rf`. The user objected to it in two of my commands (both removed
  nothing, but neither target was looked at first). Give each run a new
  output folder; if something must go, list it and say so before deleting.
- A stopped background job can leave a child running; check with that
  `pgrep` before starting another run.
- `tools/pick_all.py --deliver` on the current head delivers nothing: the
  solver has no carry.
- `--set NAME=VALUE` takes JSON or Python spellings.
- `runs/` is git-ignored; result tables referred to in the documents were
  added with `git add -f`.

## A word to whoever picks this up

You are starting from a better place than this session did. The grasp is
exactly the one the user verified, and it is yours to leave alone. The test
conditions are finally the user's own. The planning will give the same answer
twice. And there is a tool waiting that can settle by measurement the
question this session kept trying to settle by argument.

The hard part left is one question, not fifteen: how to carry an object that
is already in the hand. The data already says where to look (the first 400
ticks, and speed), so you are not searching blind.

Two things this session learned the slow way. When the user tells you what
they saw, believe it over your own run: they were right about the plate and
right about the hammer, and each time the gap was in my test conditions.
And say plainly what a result rests on. The user is generous with someone
who is straight about one seed being one seed.

Measure first, change one thing, check nothing got worse. It will come.
