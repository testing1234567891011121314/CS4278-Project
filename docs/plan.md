# Plan: five testable segments

The rewrite is built in segments. Each has its own states, library modules,
entry and exit conditions, and tests, so it can be built and checked without
the segments after it. Requirement ids refer to `docs/requirements.md`.

| Segment | States (`solver.State`) | Library modules |
|---|---|---|
| 0. Test bench | none | `tools/`, `tests/support.py` |
| 1. Go to object | `SETTLE`, `GO_TO_OBJECT` | `robot`, `world`, `grasping/candidates`, `navigation/grid`, `navigation/a_star`, `navigation/parking`, `navigation/base_follower` |
| 2. Attempt grasp | `GRASP` | `planning_sim`, `collision`, `grasping/ik`, `grasping/arm_planner`, `grasping/executor` |
| 3. Navigate to exit | `NAVIGATE`, `REACH` | `navigation/carry`, `navigation/reach`, plus the grid, path search and base follower from segment 1 |
| 4. Retry | `RETRY`, `STOPPED` | the state machine in `solver.py` |

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

**Objects with no antipodal hold (R13).** These need experiments before the
candidate interface is fixed, because they decide what a candidate can be.

| Object | Why a pinch fails | To try, in order |
|---|---|---|
| master chef can | 103.0 mm against a 102.85 mm gap; upright; 1.054 kg | (a) press the open gripper over it and measure whether the 0.15 mm overlap holds it; (b) slide it to overhang the table edge and lift it resting on the fingers and palm; (c) tip it and repeat (b) lying down |
| plate | 7 mm flange, 10 to 16 N of grip | flange hold with the lower finger under the rim (worked in the old code once the base stopped jamming on the table corner) |
| large marker | wedged in the tabletop; thrown out when the arm comes near | pick it where it lands after being thrown out; approach so that it is thrown toward open floor |

Each experiment is a throwaway script that reports lifted or not, hold force,
and whether the object survives 200 ticks of base motion.

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
