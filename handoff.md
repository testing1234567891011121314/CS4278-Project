# Handoff: Pick-Navigation-Reach debugging

Read `fixes.md` first. It explains every change made so far and why.
This file covers where things stand, how to test, and what to do next.

## Where things stand

- **Branch:** `fix/pick-nav-reach`, one commit on top of `last` (`ac3da6a`).
  - It has **not been pushed**: the previous session had no GitHub access.
  - Push it, then open a PR into `last`.
- **Files changed:**
  - `solver.py`
  - `custom_libraries/grasp.py`
  - `custom_libraries/navigate_to_grasp.py`
  - `fixes.md`, `handoff.md`, and the scripts in `tools/` (new)
- **Results.** From headless runs on seeds 42 and 7. Runs are **not deterministic**, so judge by rates, not single runs.

  | Stage | Result |
  |---|---|
  | Valid grasp start state | 30 / 30 runs (15 objects × 2 seeds) |
  | Pick (lifted, held 100 steps) | 9 / 30 in the full sweep. The five small objects picked 10 / 10 in the later end-to-end runs. |
  | Full task (object within 10 cm of goal) | 2 / 10: banana and strawberry, both seed 42 |

  Per-object picture from the last sweep:
  - **Pick reliably:** banana (5), strawberry (6), apple (7), foam brick (14), tomato soup can (2).
  - **Plan found, but execution fails** (object pushed or slips): mustard (3), hammer (13), potted meat (4).
  - **No plan** (`ik_fail` / `transit_fail`): sugar box (1), large marker (12).
  - **Not re-tested after the fixes:**
    - master chef can (0): the wide-object opening fallback was added after the sweep. It planned again, but no full run was done.
    - bowl (9) and plate (11): rim-grasp fix.
  - **Never leaves state 0, no error:** bleach cleanser (8). Not investigated.
  - **Mug (10):** the environment drops it on the floor after `env.reset()`, so it is unreachable and the solver goes to state 4.

## How to run and test

Setup:
```bash
pip install pybullet numpy scipy trimesh matplotlib
```

Single headless episode. This prints a `RESULT:{json}` line with:
- the states visited and the final state;
- `grasp_error`, `lift_cm` and `planner_stats` (per-reason reject counts);
- `held_100`, `min_goal_dist` and `stopped`.
```bash
python tools/run_one.py . --object 5 --seed 42 --max-steps 16000 --maze-copy /tmp/m_5_42.urdf
```

Notes on the runner:
- It stops early when the object is within 10 cm of the goal, or when the solver enters state 4.
- A full run with transport takes about 9,000–10,000 steps, or 1–2 minutes on 2 cores.
- Use `--max-steps 3000` when you only care about the pick.
- Give each parallel run its own `--maze-copy` path. The env writes the maze URDF there, and parallel runs clobber each other otherwise.

Watching the carry phase:
```bash
python tools/trace_nav.py . 5 42 14000
```
It prints the base, the object, the distance to the goal, the waypoint and the reach phase every 400 ticks in states 5, 67 and 68. Edit the condition at the top of its loop to trace other states or intervals.

Sweep and classify. Logs must be named `o{idx}_s{seed}.log`:
```bash
mkdir -p logs
for s in 42 7; do for i in $(seq 0 14); do echo "$i $s"; done; done | \
  xargs -P 2 -n 2 sh -c 'timeout 600 python tools/run_one.py . --object $0 --seed $1 --max-steps 3000 --maze-copy /tmp/m_$0_$1.urdf > logs/o$0_s$1.log 2>&1'
python tools/summarize_sweep.py logs
```
It labels each run PICK_OK, START_INVALID, SEARCH_EXHAUSTED, EXEC_FAIL, CRASH, TIMEOUT or OTHER.

## State machine (solver.py)

| State | Meaning | Exit |
|---|---|---|
| 999 | Wait 200 steps for the object to land | → 0 |
| 0 | `NavigateToGrasp` decides NAVIGATE / PREPARE / GRASP | → 1 (GRASP), → 4 (no usable grasp) |
| 1 | Hold the latched pose, wait to settle, `GraspPlanner.plan()` | → 2 or 4 |
| 2 | `GraspFollower`: approach, close, lift | → 3 or 4 |
| 3 | Hold the lifted object for `hold_before_transport` ticks (60) | → 5 |
| 5 | Retreat `retreat_distance` (1.0 m) straight back, arm frozen | → 67 |
| 67 | Re-tuck into `carry_joint_targets` slowly, rebuild the map with the table as an obstacle, then A* + `CurvedNavigator` with base speed capped at `carry_speed` | → 68 when `curved_nav.status == "reached"` |
| 68 | `reach_goal()`: back off `reach_backoff_cells` (7) along the path, solve IK so the object (not the gripper) lands on the goal, move the arm slowly | holds |
| 4 | Failed; holds the last pose | terminal |

Tunables live in `Solver.__init__`:
- `hold_before_transport`
- `retreat_distance`
- `carry_speed` (4 mm/tick)
- `reach_backoff_cells`
- `travel_joint_targets` (driving to the table)
- `carry_joint_targets` (carrying through the maze)

The tuck rate while carrying is `_tuck_rate`, set to 0.003 rad/tick when entering 67.

## Things that are easy to get wrong

- **Action format.** The action is 15 absolute joint targets.
  - Base world x = `qpos[0] − 2.0`.
  - Index 2 is base yaw.
  - Index 3 is the torso.
  - Indices 6–12 are the arm.
  - Indices 13 and 14 are the fingers.
- **`env.step` sleeps** 1/240 s per substep. The tools monkeypatch `time.sleep`; do the same in any new script.
- **No integral action in the controller.** The env's position controller has gains of 0.3 and no integral term. Commanding the *measured* pose as a hold target makes the arm sag; this was bug 2. Always latch targets.
- **Navigation runs at a fixed heading.** The wall inflation uses the robot's *current* projected footprint. Any arm pose wider than about 0.3 m from the base centre seals the maze bottleneck. Always measure a new pose with `custom_libraries/wall_projection.robot_xy_polygons(env)`.
- **The held object isn't in the footprint.** The footprint ignores the object, so a long object can still brush the walls.
- **The parking map leaves the tabletop out on purpose.** `create_map(env, is_2d=True)` skips boxes whose lower z is ≥ 0.35, so the base can tuck under the table. For transport, `create_map(env, False)` is called to include them.
- **`a_star` returns `[]` on failure, not `None`.**
- **Line endings.** `grasp.py` uses CRLF; the other files use LF. Preserve them when editing with scripts.

## Recommended next steps, in priority order

1. **Make the grasp hold through transport.** This is the biggest win: 8 of the 10 end-to-end runs picked the object but lost it. They drop it during the re-tuck, the navigation, or the final arm motion. Ideas:
   - squeeze harder once lifted (raise the finger command below the contact width in state 3);
   - prefer grasps near the object's centroid, and closing directions that are horizontal;
   - check finger contact each tick, and on loss stop the carry and go back to re-grasp rather than continuing empty.
2. **Verify the untested fixes** with a full sweep: master chef can, bowl and plate.
3. **Fix the planner for tall or long objects** (sugar box, large marker, potted meat). Their failures are `ik_fail` plus `transit_fail` where the forearm or wrist hits the table. Things to look at:
   - candidate ranking;
   - side (horizontal) approaches for boxes;
   - the base standoff distance.
4. **Investigate the bleach cleanser** sitting in state 0. Add a print of `decision` / `decision_state` in solver state 0.
5. **Episode length.** Check whether the grader has a step limit. Transport takes about 10k steps at `carry_speed` = 4 mm/tick, so raise the speed carefully once the grasp is firmer.
6. **Housekeeping**, if the team agrees:
   - the committed `pnr/` virtualenv;
   - the deleted `requirements.txt` and tests on `last`;
   - the stray `test.patch`.
