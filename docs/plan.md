# Plan: five testable segments

The rewrite is built in segments. Each has its own states, library modules,
entry and exit conditions, and tests, so it can be built and checked without
the segments after it. Requirement ids refer to `docs/requirements.md`.

| Segment | States (`solver.State`) | Library modules |
|---|---|---|
| 0. Test bench | none | `tools/`, `tests/support.py` |
| 1. Go to object | `SETTLE`, `GO_TO_OBJECT` | `robot`, `world`, `grasping/candidates`, `navigation/grid`, `navigation/a_star`, `navigation/parking`, `navigation/base_follower` |
| 2. Attempt grasp | `GRASP`, `DRAG` | `planning_sim`, `collision`, `grasping/ik`, `grasping/arm_planner`, `grasping/executor` |
| 3. Navigate to exit | `BACK_OFF`, `NAVIGATE`, `REACH` | `navigation/carry`, `navigation/reach`, plus the grid, path search and base follower from segment 1 |
| 4. Retry | `RETRY`, `STOPPED` | the state machine in `solver.py` |

**Status (2026-11-20).** Segments 1 and 2 and the back-off of segment 3 are
written and run for all 15 objects on one maze by `tools/pick_all.py`; the
per-object results and every grasp change are in `GRASP.md`. `NAVIGATE` and
`REACH` are written but not finished: 11 of 15 objects are delivered in one
run (`runs/deliver_11`; `NAVIGATE.md` compares every attempt). Of
segment 0, only the headless env with a seeded cloud (`tools/trial.py`) and
the pick measurement (`tools/pick_all.py`) exist; `tools/score.py` and
`tools/batch.py` are still the old solver's and do not run against this one.

A segment is done when its fast tests pass, its simulator test passes, and the
paired benchmark against the reference loses no points on the objects the
segment covers.

Running the tests:

- Fast tests (no simulator): `pnr/bin/python -m unittest discover -s tests -t . -v`
- Simulator tests are skipped unless `PNR_SIM_TESTS=1`. Run them one process
  at a time under the memory cap:
  `flock /home/shared/pnr_sim.lock systemd-run --user --scope --quiet -p MemoryMax=2000M -p MemorySwapMax=0 -E PNR_SIM_TESTS=1 pnr/bin/python -m unittest discover -s tests -t . -v`

## Segment 0: test bench

Nothing can be judged before this works (R27, R28, R29).

- `tests/support.py`: a headless env for tests, with the cloud seeded from
  outside the env.
- `tools/score.py`: read only `solver.state` and `solver.report`; measure the
  lift, hold, pose error, drop and delivery from the simulator; default to
  3,000 ticks; measure a floor pick's lift against the tabletop; write a score
  line when the trial crashes; record the settings; accept the cloud seed.
- `tools/batch.py`: leave `old/` out of the snapshot.
- The reference numbers: run `4fafaea` (in `old/`) on the same seeded draws at
  3,000 ticks.

Done when the scorer runs the stub solver to a `stopped` score line, and the
reference table for `4fafaea` is reproduced.

## Segment 1: go to object

| | |
|---|---|
| Starts | at the first tick of a trial, or when a retry begins a new attempt |
| Does | waits for the object to rest; reads the scene; generates grasp candidates from the cloud; chooses a parking pose from which a candidate is reachable; plans a base path; drives it; turns to face the object |
| Ends well | base parked and still, at least one candidate within reach, object not disturbed → `GRASP` |
| Fails to `RETRY` | no candidates; no parking pose; no route; base blocked; tick budget spent |
| Requirements | R4, R7, R14, R19, R24 |

Fast tests:
- `test_grasp_candidates.py`: antipodal pairs on a synthetic box, cylinder and
  sphere; nothing wider than the gripper; same seed gives the same list.
- `test_grid_a_star.py`: walls from cube positions; disc inflation; shortest
  path on a small maze; `None` when there is no route; start inside an
  inflated cell does not deadlock.

Simulator test (`test_go_to_object.py`): for a table object and a floor
object, the base parks within the budget, touches neither table nor walls, and
the object has not moved.

## Segment 2: attempt grasp

| | |
|---|---|
| Starts | parked, with the attempt's candidates |
| Does | rejects candidates cheaply (finger opening, palm inside the object); solves inverse kinematics; plans the transit, the straight approach and the lift; executes; closes; verifies the hold; lifts; holds 50 ticks |
| Ends well | object lifted at least 10 cm and held 50 ticks → `NAVIGATE` |
| Fails to `RETRY` | no plan; object moved before the close; hold not verified; object lost in the lift or the hold; tick budget spent |
| Requirements | R7 to R13, R22, R26 |

Fast tests:
- `test_ik.py`: solutions reproduce poses made by forward kinematics, respect
  joint limits, return `None` for unreachable poses.
- `test_grasp_candidates.py`: the cheap rejections.

Simulator test (`test_planning_sim.py`): the copy matches the live scene, and a
planning call leaves the live simulator's state identical (R30, R31).

Simulator test (`test_attempt_grasp.py`): starting from a known parking pose,
the foam brick is lifted by tick 600, at least 10 cm, held 50 ticks, with the
pose error reported.

**Objects with no antipodal hold (R13).** What was found for each is in
`GRASP.md`. In short: the master chef can is held by a tight fit (the fully
open hand pushed over it); the bowl by a pinch of its rim; the plate is raked
to the table's edge and gripped where it overhangs; the marker is thrown out
of the table by the simulator and picked from the floor where it lands.

## Segment 3: navigate to exit

| | |
|---|---|
| Starts | object held after the 50-tick hold |
| Does | backs away from the table; chooses a carry posture; plans the route from `cube_positions`; drives with ramped speed; at the exit, moves the arm so the object, not the gripper, is at the goal |
| Ends well | object within 10 cm of the goal → `DONE`, pose latched |
| Fails to `RETRY` | object lost; no route; base blocked; goal out of reach; tick budget spent |
| Requirements | R14 to R17, R19, R22, R23 |

Fast tests:
- `test_grid_a_star.py`: a route exists through the generated maze for several
  maze seeds.
- `test_navigation.py`: the per-tick base step never exceeds the speed limit;
  the follower stops within tolerance of the path's end.

Simulator test (`test_navigate_to_exit.py`): with the foam brick placed in
the gripper by the test, the object is delivered within the budget in two
mazes, without a drop. A second case drives the empty gripper to the goal.

## Segment 4: retry

| | |
|---|---|
| Starts | any failure exit, any exception, any state timeout |
| Does | records why; decides whether enough ticks remain; starts a fresh attempt from where the object now is (table or floor), or stops |
| Ends | `SETTLE` with a new attempt, or `STOPPED` with a latched pose |
| Requirements | R18 to R23 |

Fast tests (`test_solver_contract.py`, with a fake env): the action is always
a finite (15,) array; an exception in any state does not escape; a state that
overruns its budget fails; a new attempt shares nothing with the last; the
solver is `STOPPED` at tick 3,000.

Simulator test (`test_retry.py`): the test opens the gripper during the carry;
the solver picks the object up from the floor and delivers it, or stops
cleanly inside the budget.

## Order of work

1. Segment 0.
2. The state machine's spine from segment 4 (already in the skeleton), so
   every later segment runs inside the guard and the budgets.
3. Segment 1, then the experiments for the three hard objects, then segment 2.
4. Segment 3.
5. The rest of segment 4.

## Segment 3 from here: the pick is fixed, the travel pose is investigated

Written 2026-11-20, after the first eleven carry attempts (`NAVIGATE.md`).
It replaces "chooses a carry posture" in segment 3 above with a plan.

**Rule.** The grasp and the travel pose are two problems. The grasp is chosen
only to get the object off the table. A loss during the carry is fixed after
the lift, never by choosing a different grasp. An optimal grasp is not an
optimal travel pose.

**Why the rule.** Between the pick commit (`b701e73`) and the first carry
commit (`f09377d`) I changed which grasp is chosen in order to help the carry.
Under `run.sh` the plate, which the pick commit grasps, is no longer grasped,
and the hammer is not gripped the way the change intended. My evidence for
those changes was one fixed cloud draw in a different maze from `run.sh`'s.

**Target.** 15 of 15 delivered; a re-pick only where there is no choice.

**Clean revert (user, 2026-11-20).** The pick path is byte for byte that of
`b701e73` (same git hashes), not the same behaviour kept behind switches.
The solver therefore has no carry at present. Counting planning work instead
of time comes back as a separate change on top, and nothing else.

### Order of work (set by the user, 2026-11-20)

1. **Measure the pick against the pass bar.** The pick as restored, with
   work-based planning limits: all 15 objects on five cloud seeds in
   `run.sh`'s maze. Per object: lifted 10 cm and held 50 ticks by tick 600,
   out of five. The bar is 4 of 5 (`docs/requirements.md`). No fixing yet.
2. **Find the best travel pose** (steps 3 and 4 below), with the grasp left
   exactly as the pick makes it.
3. **Then return to the grasp** for the objects that miss 4 of 5.

An earlier "15 of 15" for the pick was two cloud draws in the env's own maze.
It is not a rate and is not used as a baseline. Results are quoted with their
seeds, maze and number of trials.

### Step 1. Put the pick back, and prove it

| | |
|---|---|
| Does | Up to the end of the back-off, the solver chooses and executes grasps exactly as `b701e73` does. Removed from that path: the centre-of-mass ranking, the "swings" tier, the grip under the hammer's head, the changed order of tight fits, and the arm solver's stuck-joint handling (kept only for the hold and the final reach). |
| Passes when | On the same cloud seeds in `run.sh`'s maze, every one of the 15 objects has the same lift tick, hold result and back-off as the exported `b701e73` code (`runs/tmp/pre_nav/`). Two cloud seeds at least. Any difference means the restore is incomplete. |
| Not allowed | Judging by "looks as good". The check is equality. |

### Step 2. Test under the conditions that are graded and that the user runs

| | |
|---|---|
| Does | `tools/pick_all.py` gets `run.sh`'s maze (built as `run_batch.py` builds it) as an option, so a full run costs no video and about a minute per object. |
| Conditions from now on | `run.sh`'s maze; several cloud seeds per comparison; old code against new on the same seeds. One fixed draw in the env's default maze is no longer accepted as evidence for anything. |
| Baseline | With the pick restored and the carry as committed: delivered count per object over 3 cloud seeds. Every later change is compared with this. |

### Step 3. Survey travel poses

A measurement, not a design. `tools/travel_poses.py`:

1. Pick the object once with the restored pick. Save the simulator's state at
   the end of the back-off. (Test equipment only: the solver itself still
   never writes to the simulator, R30.)
2. For each pose in the table below, restore that state, move the hand into
   the pose slowly, and apply one fixed stimulus built from the carry's own
   code: a turn on the spot, then 2 m straight at carry speed.
3. Record what happened.

| Variable | Values surveyed |
|---|---|
| Hand elevation | pointing down; 45 degrees down and forward; level; 45 degrees up and forward |
| Pads' line | level; vertical (one finger under the object) |
| Pads' line against the travel (hand pointing down) | along; across |
| Arm | folded onto the base's centre line (the unfolded arm does not pass the maze); plus the pose as lifted, as the baseline |

| Recorded per object and pose | |
|---|---|
| Reachable | whether the arm can take the pose, and whether the way into it is clear |
| Lost | no, or in which part: taking the pose, the turn, the drive; and after how far |
| Slip per metre | along the fingers, along the pads' line, across both |
| Turn in the hand | degrees, about each of the hand's axes |

Output: one table, 15 objects by about nine poses, in `runs/travel_NN/` and
summarised in `NAVIGATE.md`. No pose is favoured beforehand, including the
side hold.

Fixed by the user for every pose (2026-11-20): **the base turns with the
route.** Its heading follows the travel, which is what keeps the gripper
lined up with the direction of travel. A carry in which the base keeps one
heading and slides sideways was proposed and rejected for that reason. The
survey therefore adds, to the poses above, the hand turned about the vertical
so the pads' line lies along the base's heading ("pads forward"), with the
wrist left alone during the drive, at 0.70 m and 0.40 m from the base's
centre; and as a control the same with the line across the heading.

**The back-off is not to be hard-coded** (user, same day): after an object is
picked up off the floor, backing straight away from where it lay has caused
collisions. The retreat is to be chosen from the scene: the direction and
distance that are clear, or no retreat at all where folding the arm in
already clears whatever the hand was over. To be done with step 4, and
checked on re-picks from the floor in particular.

### Step 4. Turn the table into the hold, and check it on whole trials

| | |
|---|---|
| Does | For each object the pose (or the rule that picks the pose from what can be measured after the lift: where the centre of mass lies from the pads' line, the hand's direction, the object's size) that the survey shows carries best. |
| Passes when | On 3 cloud seeds in `run.sh`'s maze the delivered count is higher than the step 2 baseline and no object that was delivered there is lost. A rule that helps one object and costs another is not kept; both go in the change log. |
| If an object has no pose that carries it | Reported as a finding with its numbers. The grasp is not changed to fix it. What remains to try for it is speed, turn rate, the route, and recovery. |

### Step 5. What is left after the pose

In this order, each judged as in step 4: speed and turn rate per object
(from the survey's slip per metre); the turn on the spot at the start of the
carry; recovery after a drop (the plate on the floor is currently never
picked up again).

### Records

`NAVIGATE.md` gets every attempt, as now. `GRASP.md` gets a note that the
grasp-side changes of its sections 6 and 7 are withdrawn. The library of 63
further objects (`runs/library_01`) is a second test set for the result, not
something to tune on.

