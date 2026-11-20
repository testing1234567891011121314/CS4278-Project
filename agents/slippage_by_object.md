# Slippage by object

How far each of the 15 YCB objects slides in the gripper, in which direction and
phase, and why it slips or is lost. The code is the frozen snapshot that made
the traces: fingers-up carry pose, full-force squeeze, carry at 36 mm per step
commanded (about 30 mm actual) with a 2 mm per step ramp.

## Axes, phases and sources

**Gripper axes** (gripper frame, origin at the centre of the finger pad):

- **x** runs along the finger, toward the fingertips. The pad runs from
  x = -31.5 to +31.5 mm. In the carry pose the gripper points straight up, so
  +x is **up** and -x is **down toward the palm**. Before the swing (hold,
  retreat), x points wherever the grasp approach pointed, so up/down is not used
  for those phases.
- **y** is the closing axis. In the carry pose the wrist roll keeps it along
  the direction of travel.
- **z** runs across the pad, along its 24 mm width (edges at z = ±12 mm). In the
  carry pose it is horizontal and at right angles to travel. "dz" means sliding
  sideways toward the 24 mm edge of the pad.

**Drift** (dx, dy, dz) is the displacement, in this frame, of the object's
origin. For these URDFs the origin is the centre of the mesh bounding box and
also the centre of mass. A rotation of the object about the contact therefore
shows up as drift too.

**Phases**, taken from the state column, dbase_mm and the base heading:

| Phase | How it was found | Typical length |
|---|---|---|
| hold | state 3; not in the traces (only `gs_*` `slip_hold`) | 50-60 steps |
| retreat | state 5; base ramps 0 → 30 mm per step and back | 46 steps |
| arm swing | state 67 before the base first moves (> 1 mm per step for 3 steps). It ends with the wrist-roll alignment at 0.1 rad per step (arm_qvel ≈ 2.08) | 225-390 steps |
| accel from rest | state 67, base speed rising by > 0.5 mm per step | 30-65 steps per run |
| corners | state 67, heading change > 0.01 rad per step (averaged over ±3 steps) | 190-235 steps |
| cruising | state 67, straight, steady, > 20 mm per step | about 550 steps |
| reach | state 68 | 250-330 steps |

**Sources.**

- `logs/all/t_*` are the 30 main traces. `logs/c17_*` are 12 more traces of
  objects 2, 5, 6, 7 and 14.
- `logs/gs_*` are 30 pick-and-swing runs. They give grasp geometry at lift and
  slip from the lift, which is the only hold-phase slip data.
- I added 8 `tools/score.py` runs and 4 probe runs that print the
  grasp-selection error. Outputs are in `S/slip_tmp/sc/`; the analysis scripts
  are `S/slip_tmp/an.py`, `pr.py` and `det.py`.

**Data vs inference.** Statements marked **Data** are read directly from those
files. Statements marked **Inference** are my reading of the mechanism, using the
measured physics: pads 63 × 24 mm, μ ≈ 0.25, no torsional friction, and a per-step
peak kick of about 15,800 × (displacement per step). At 30 mm per step that is
about 470 m/s² along travel.

**Two corrections to the brief.**

- **Delivered count.** The RESULT lines give **7 of 30** deliveries, not 8: soup
  can ×2, foam brick ×2, banana s42, strawberry s7, apple s42.
- **Drift reference.** `tools/trace_carry.py` sets the reference on the first
  logged row, which is the first step of the retreat (state 5), not the lift.
  Hold-phase slip is therefore not in the traces.

## Overview

"Runs" counts t + c17 traces. "Slip" is the drift norm at the end of the arm
swing, then at delivery or at loss.

| # | Object | Runs | Delivered | Typical slip | Main cause (Data / Inference) |
|---|---|---|---|---|---|
| 0 | master chef can | 2 | 0 | none (never grasped) | 103 mm diameter is wider than the ~95 mm usable opening; planning fails on IK/approach |
| 1 | sugar box | 2 | 0 | 14-18 mm by swing end; lost at 18 and 90 mm | Heaviest carried (0.64 kg): kicks reach the friction capacity even at 300 N |
| 2 | tomato soup can | 4 | 4 | 2-12 mm at swing end, 3-24 mm at delivery, almost all dz | Likely a pivot about the closing axis: CoM sits 21-48 mm toward the fingertips from the contact centre |
| 3 | mustard bottle | 2 | 0 | none | No grasp plan (60 tried: 38 IK, 21 approach, 1 transit failures) |
| 4 | potted meat can | 2 | 0 | 25 mm by swing end; lost at 81 mm | Slid sideways across the pads during the swing and the first 30 steps of driving |
| 5 | banana | 4 | 2 | 5-13 mm at swing end, then 23-36 mm down toward the palm while cruising | Low grip (~125 N, 40 mm wide); short line contact 2-5 mm from the fingertip end; per-step kick unloads a finger by ~32 % |
| 6 | strawberry | 4 | 1 | 3-7 mm at swing end; lost at 12-21 mm | Creeps even in the stationary hold; lost when the point contact reaches a pad edge, mostly in corners |
| 7 | apple | 5 | 4 | 1-14 mm at swing end, 2-23 mm at delivery | Single-point contacts; s7 crept 18 mm sideways over 600 driving steps, then was squeezed out |
| 8 | bleach cleanser | 2 | 0 | none | Never leaves state 0 (lies on its side; navigation never finishes) |
| 9 | bowl | 2 | 0 | none | "No antipodal grasp candidates found" at step 63-67 |
| 10 | mug | 2 | 0 | none | Falls to the floor; "grasp height outside the coarse height range" |
| 11 | plate | 2 | 0 | none | "No antipodal grasp candidates found" at step 63 |
| 12 | large marker | 2 | 0 | none | No grasp plan: all 60 tried fail the Cartesian approach |
| 13 | hammer | 2 | 0 | 17-19 mm before loss | Low grip (115 N) on a long, high-inertia object; lost in the swing (s7) and at the wrist-roll alignment (s42) |
| 14 | foam brick | 5 | 3 | 3-4 mm at swing end in good runs; 23-32 mm in the 2 losses | Good grasp (flat patch); both losses (seed 42, c17) had already slid 23-32 mm in the retreat and swing |

**Grip force** (Data). It is set by object width alone: about **6.6 N per mm of
finger opening per finger**, because the command is 0 (shut).

| Object | Grip force per finger |
|---|---|
| banana | 123-138 N |
| hammer | 114-116 N |
| strawberry | 127-146 N |
| foam brick, 51 mm face | 158-169 N |
| potted meat can | 180 N |
| soup can | 206-229 N |
| apple | 196-256 N |
| foam brick, 78 mm face | 209-274 N |
| sugar box (92 mm side) | 296-303 N, above the "wide 200-250 N" figure |

---

## 0. Master chef can

**Shape.** Cylinder 103 × 102 × 140 mm, 1.05 kg, the heaviest object.

**Runs.** 2 traces, 0 grasped.
- s42 went to state 4 at step 68 with no error string. The handoff notes the
  same step-68 failure, there as "no grasp candidates".
- s7 failed at step 114 with "No plan found within the grasp search budget".

**Slip.** No data; the can is never carried.

**What fails (Data).** A `score.py` rerun on s42 behaved differently from the
trace. It found 500 candidates, tried 60, and failed with 26 IK failures and 34
approach failures at both pad depths ("search_exhausted").

**Inference.** The planner accepts widths up to 102.85 mm − 8 mm clearance =
94.85 mm. Both cross-sections of the can are ≥ 102 mm, so the only pairs that
fit are off-centre chords or rim features. Those are awkward to reach and far
from the centre of mass. Even if grasped, 1.05 kg is the worst case in the
synthesis table: at 30 mm per step the per-step kick would unload a finger by
about m·a/(2N) ≈ 1.0 at 250 N. That is a full unload, so it would very likely
slip.

## 1. Sugar box

**Shape.** Box 45 × 92 × 176 mm, 0.64 kg. Held across the 92 mm side at
296-303 N per finger, the highest grip of any object.

**Runs.** 2, both dropped in state 67.

**s42 (lost at step 629, mid-swing, base still).**
- Retreat: 3 mm.
- Swing: the box slid steadily, +7 mm toward the fingertips and -13 mm sideways
  across the pad (dz). It reached (10, -1, -14) mm at step 628.
- Contact then vanished in one step with no force warning. Force was 296-302 N
  throughout.

**s7 (lost at step 1036, mid-maze).**
- Swing: it slid 13 mm down toward the palm (dx -13) by the swing end. A
  one-step force dip to 83 N occurred in the swing.
- Driving: dz grew from +5 to -84 mm, mostly in corners (-55 mm over 139 corner
  steps), then cruising (-25 mm over 106 steps), plus -8 mm in 31 acceleration
  steps. dx went to -31 (down).
- During driving one finger carried 280 N and the other 320-380 N. That is a
  steady force along the closing axis.
- The box is 92 × 176 mm in the pad plane, so the pads could slide that far
  along its face before running off it.

**Why (Inference).** This is the heaviest object that gets carried. Gravity is
not the problem: 6.3 N against a friction capacity 2μN ≈ 150 N.

- At 30 mm per step each kick along the closing axis is about 0.64 × 470 ≈ 300 N.
  That is as large as the grip itself, so the trailing finger loses about half
  its normal force every step (u_n ≈ 0.50, the second-highest of all objects).
- The synthesis estimates 220-470 m/s² grasp-centre kicks during the
  joint-space swing. That gives 140-300 N in the pad plane against 150 N of
  capacity, which is at or past the slip limit even at full grip.
- Both runs show a steady slide during the swing. s42 never got to drive. Mass
  against friction capacity is the best-supported cause. The force-imbalance
  data in s7 also points at the closing-axis load.

## 2. Tomato soup can

**Shape.** Cylinder 68 × 102 mm, 0.32 kg. Held across the 68-70 mm diameter
with the can's axis along x.

**Grip.** 206-229 N per finger.

**Runs.** 4 (t s42, s7; c17 s42, s7), **4 delivered**.

**Contact geometry (Data, 6 gs runs).**
- The pads touch along a line along x (contact spans up to 50-57 mm along x,
  0-18 mm across).
- The force-weighted contact centre is at x = -22 to +4 mm, while the centre of
  mass is at x = 24-29 mm. The CoM is therefore **21-48 mm toward the
  fingertips** from the contact centre: in the carry pose most of the can stands
  above the fingers.
- Hold slip ≤ 2.3 mm.

**Slip (Data).**

| Phase | Slip |
|---|---|
| retreat | ≤ 1.5 mm |
| swing | 2-12 mm, almost all dz (-12.1, -4.6, -11.6, -1.0 mm) |
| driving | dz grew to 17-24 mm in three runs, 3 mm in the fourth (c17 s42) |
| reach | 1-3 mm |

- During driving, dx and dy stayed within 4 mm.
- Over all driving steps, dz changes correlate with the estimated lateral kick
  of base turning (about 20,760 × v × dψ, peaks 100-130 m/s²). The correlation
  is 0.39-0.54 in three runs, the highest of any object. Turning steps carry
  33-47 mm of summed |Δdz|, against 9-27 mm on straights.

**Why (Inference).** A 24 mm sideways translation of a line contact on a 24 mm
pad would have taken the contact off the pad. The can was still held, so the dz
drift is most likely a **pivot of the can about the closing axis** (torsion),
not sliding.

- The CoM is 21-48 mm from the contact centre along x, so a pivot of about 30°
  moves it about 24 mm in z.
- Torsion demand in a corner: m × a_lat × r_x ≈ 0.32 × 100 × 0.035 ≈ 1.1 N·m.
- Torsion capacity: μN(ρ1 + ρ2) ≈ 0.25 × 220 × 0.025 ≈ 1.4 N·m, taking ρ ≈ L/4
  for a ~50 mm line.
- So corners bring it near the torsion limit, which fits the correlation with
  turning.
- It is never lost because the pivot is bounded and the line contact stays on
  the pads. Translation along y is all normal force.

## 3. Mustard bottle

**Shape.** 96 × 58 × 191 mm, 0.61 kg.

**Runs.** 2. Both stop with "No plan found within the grasp search budget" at
steps 340-379, before any grasp. No slip data.

**What fails (Data, `score.py` s42).** 500 candidates; 60 tried; 38 IK
failures, 21 approach failures, 1 transit failure; exhausted at both pad depths
(10 and 25 mm).

**Inference.** The 58 mm side is the only one that fits comfortably. Most
candidates on it are rejected by IK from the parking spot. I cannot say more
from these statistics.

## 4. Potted meat can

**Shape.** Box-like 102 × 58 × 84 mm, 0.35 kg. Held across about 54 mm at
180 N per finger.

**Runs.** 2.
- s42: dropped at step 803 in state 67.
- s7: "arm reached lift pose but the object did not rise" at step 344.

**s42 (Data).**
- Retreat: 7 mm, (4.5, 3.2, 4.3). This is the largest retreat slip apart from
  the banana loss and one foam brick.
- Swing: it slid 28 mm sideways across the pad (dz -16 by step 760, base
  still). The wrist-roll alignment at the end of the swing (steps 769-776) added
  another -13 mm of dz.
- Driving: in the first 14 steps of acceleration from rest it went another
  -17 mm in dx (down toward the palm) and -23 mm in dz.
- Then cruising, and loss 28 steps after the base first moved, at
  (-31, 10, -74) mm.

**s7.** No slip data. A `score.py` rerun of s7 lifted the can normally
(14.4 cm, held 50 steps), so the "did not rise" failure does not reproduce. It
looks like run-to-run variation in the grasp, not a property of the object.

**Why (Inference).** At 180 N the friction capacity in the pad plane is 2μN =
90 N.

- Swing kicks estimated at 220 m/s² demand 0.35 × 220 ≈ 78 N (u ≈ 0.86).
- Along travel the per-step kick unloads a finger by u_n ≈ 0.47 at 30 mm per
  step.
- Mass relative to grip is the second-worst after the sugar box. It slid
  steadily in the swing.
- The flat faces let the pads travel 70+ mm along the box before losing it. One
  run is not enough to separate the swing kicks from the roll as the main driver.

## 5. Banana

**Shape.** Curved, 198 × 69 × 37 mm, 0.17 kg. Held across about 40 mm at
123-138 N per finger, the narrowest grasp of the carried objects.

**Runs.** 4, 2 delivered.

| Run | Outcome |
|---|---|
| t s42 | delivered |
| t s7 | lost in the retreat at step 386 |
| c17 s42 | lost at step 1161 mid-maze |
| c17 s7 | delivered |

The two seeds swap outcomes between the t and c17 batches.

**Contact geometry (Data, 6 gs runs).**
- Every grasp uses the 25 mm pad depth.
- The contact is a short line *across* the pad: 2-4 mm along x, 19-23 mm along
  z. It sits at x = 25-29 mm, only **2-5 mm from the fingertip end** of the pad.
- The CoM is offset 11-12 mm along the closing axis and 2-4 mm toward the palm.
- Hold slip is 0.4-1.2 mm toward the fingertips in all 6.
- 2 of 6 gs runs report "lost" during the swing (steps 441 and 596). The traces
  show no single-step contact flicker anywhere, so these may be real losses, but
  the gs script cannot tell a flicker from a drop.

**Slip (Data).**
- **t s7, retreat.** Steady until the base reached 27-30 mm per step (step 380).
  Then, in 5 steps, it moved +12 mm toward the fingertips and -13 mm in dz. Both
  finger forces fell from 125 to about 50 N, and contact was gone at step 386.
- **Swing**, other runs: 5-13 mm, mostly toward the fingertips (dx +5 to +8),
  except c17 s42 (-10 dx, -8 dz).
- **Driving.** The common pattern is a creep **down toward the palm** while
  cruising:

  | Run | Cruising dx | Corners dx |
  |---|---|---|
  | t s42 | -11 mm over 557 steps | -12 mm |
  | c17 s7 | -34 mm | |
  | c17 s42 | -45 mm over 297 steps | |

  In c17 s42 dz also went to -79 mm. That means the banana slid lengthwise
  through the fingers. The creep sped up over the last 60 steps before the loss,
  on a straight at 30 mm per step, with the force steady until about 20 steps
  before.
- Delivered runs ended at 26-31 mm of drift.

**Why (Inference).**
- The banana gets the least force of the light objects, because force scales
  with width.
- Along the travel axis, each 30 mm per step kick (about 0.17 × 470 ≈ 80 N)
  takes about 32 % of the normal force off the trailing finger (u_n ≈ 0.32).
  That is right at the ~0.3 creep threshold from the earlier calibration. A
  ratchet under these repeated unloadings, with gravity pulling down, fits the
  steady downward creep on straights.
- Holding the banana at the very fingertip end of the pad with a short line of
  contact leaves almost no pad beyond the contact and little torsional radius.
- The retreat loss happened exactly when the speed reached its maximum,
  consistent with kick size.
- Caveat: the soup can (u_n ≈ 0.34) and apple (0.24) do not creep down, so u_n
  alone does not predict this. The short contact line at the pad tip is the
  difference I can point to.

## 6. Strawberry

**Shape.** Roughly conical, 47 × 43 × 46 mm, 0.048 kg (the lightest carried).
Held across 41-46 mm at 127-146 N per finger.

**Runs.** 4, 1 delivered.

| Run | Outcome |
|---|---|
| t s42 | lost at step 1114 |
| t s7 | delivered |
| c17 s42 | lost at step 1098 |
| c17 s7 | lost at step 977 |

**Contact geometry (Data, gs).**
- Contacts are 2-6 points per pad with small spans: near-point contact. They sit
  at x = 20-24 mm, 7.5-11 mm from the fingertip end.
- CoM offset from the contact centre ≤ 6 mm.
- **Hold slip, robot stationary.** 4 of 6 gs runs slid 1.9-3.7 mm in z, and one
  (42c) slid -8.9 mm in x. That last run was later "lost" at step 744. The
  strawberry is the only object that routinely slides while nothing moves.

**Slip (Data).**

| Phase | Slip |
|---|---|
| retreat | ≤ 0.5 mm |
| swing | 3-7 mm (dx +3 to +4.5 up, dz ±3 to 6) |

- Losses all happen **in corners**, with the wrist roll active (arm_qvel 1-2):
  - t s42: dz -10 → -18.6 over the last 50 corner steps.
  - c17 s7: dz 15 → 16.6.
  - c17 s42: dx +9 → +17.7. It popped out past the fingertips in one step.
- In t s42, corners account for -9.9 of the -18.6 mm dz. In c17 s7, corners
  account for +13.5 of the +20.6 mm.
- The delivered run (t s7) stayed within 7 mm.

**Why (Inference).** Mass is not the issue. Kicks of 470 m/s² give 23 N against
2μN ≈ 70 N, and gravity gives 0.5 N.

- Slip in the motionless hold means the grip itself is pushing the object. On a
  tapered convex body, contact normals that are not quite antipodal turn part
  of the 140 N squeeze into a tangential push. The fingers commanded shut keep
  pushing as it moves.
- Corners add small lateral and roll disturbances that ratchet this creep.
- The object is lost once the point contact reaches a pad boundary: about 12 mm
  sideways, or the fingertip end about 8-11 mm above the contact. At that point
  the squeeze ejects it within 1-3 steps. Loss drifts of 12-21 mm match those
  margins plus the initial contact offset (±7 mm).

## 7. Apple

**Shape.** Near-sphere, 75 mm, 0.25 kg. Held across 73-76 mm at 196-256 N per
finger.

**Runs.** 5, 4 delivered.
- Delivered: t s42, c17 s42 ×2, c17 s7.
- Lost: t s7, at step 1550 mid-maze.

**Contact geometry (Data, gs).**
- Mostly **single-point contacts** (n = 1 on 7 of 12 pads), at the 10 mm depth,
  21-26 mm from the fingertip end.
- CoM offset ≤ 6 mm.
- 2 of 3 gs s42 runs are "lost" at steps 619-621, mid-swing. Both had slipped
  2-6 mm in the hold and retreat. The three traced s42 runs all survived the
  swing (2-12 mm), so treat the gs losses with caution: real or flicker cannot be
  told apart there.

**Slip (Data).**

| Phase | Slip |
|---|---|
| retreat | ≤ 0.7 mm |
| swing | 0.6-13.6 mm. The largest is t s42 (+11 dx toward the fingertips, +6 dz) |
| driving, delivered runs | 0-5 mm |
| reach | up to +10 mm dx (t s42) |

**t s7 loss.**
- Swing only 0.6 mm.
- Then a slow sideways creep while driving: dz 0 → 18.6 mm over about 600
  steps (about 0.03 mm per step). 28 mm of summed |Δdz| came on straights and 21
  mm in turns. Grip was steady at 233-256 N and alignment within 2°.
- At dz ≈ 18.6 the forces fell from 255 to 60 N in 3 steps and contact was lost.

**Why (Inference).**
- A point contact on a sphere follows the sphere, so creep in z walks the
  contact toward the 12 mm pad edge. The sudden 3-step collapse when the edge is
  reached is the same squeeze-ejection as the strawberry.
- What drives the creep on straights is not resolved by the trace. The closing
  axis was aligned, so travel kicks are normal forces, and loads in z are
  estimated at < 10 % of capacity.
- Each object-seed pair differs a lot run to run: t s7 0.6 mm in the swing,
  c17 s7 4.6 mm.

## 8. Bleach cleanser

**Shape.** 99 × 68 × 254 mm, 0.97 kg.

**Runs.** 2, both still in state 0 after 2,200 steps. No slip data.

**What fails (Data, probe s42 for 700 steps).**
- State 999 ran its full 200-step cap: the object never met the stillness test.
- State 0 then kept a grasp and a navigation goal but never reached GRASP in
  500 steps.
- The object sits at z = 0.652 m. The bowl's centre is at 0.641 and is 55 mm
  tall, so the bleach is **lying on its side** (68 mm thick).
- No error or "Object moved" message is printed.

**Inference.** Why navigation never completes is not visible in these logs.
Not enough data. Note that at 0.97 kg it would face u_n ≈ 0.9 per kick at
30 mm per step, if ever grasped.

## 9. Bowl

**Shape.** 161 × 161 × 55 mm, 0.033 kg, thin-walled.

**Runs.** 2, both fail at step 67. No slip data.

**What fails (Data, probe).** "Grasp selection failed: No antipodal grasp
candidates found." The bowl rests on the table (z = 0.641).

**Inference.** The 161 mm outside is far wider than the 95 mm usable opening.
The only pairs that would fit are across the thin wall. Apparently none pass the
antipodal sampler (5 mm minimum width, pad-fit test).

## 10. Mug

**Shape.** 105 × 93 × 81 mm, 0.14 kg.

**Runs.** 2, both fail at step 171. No slip data.

**What fails (Data, probe).** "Grasp height is outside the configured coarse
height range". The mug lies on the **floor** at (-2.0, -2.1, 0.045) m, and the
grasp helper needs 0.2-1.3 m. It falls off the table when spawned (the handoff
also reports it on the floor).

## 11. Plate

**Shape.** 260 × 260 × 27 mm, 0.058 kg.

**Runs.** 2, both fail at step 63. No slip data.

**What fails (Data, probe).** "No antipodal grasp candidates found." The plate
rests on the table.

**Inference.** Same as the bowl: wider than the gripper, and the rim offers no
pair the sampler accepts.

## 12. Large marker

**Shape.** Cylinder 19 × 121 mm, 0.027 kg.

**Runs.** 2, both "No plan found" at step 159. No slip data.

**What fails (Data, `score.py` s42).** 500 candidates; 60 tried; **all 60
failed the Cartesian approach** (no IK or transit failures), at both depths.

**Inference.** The marker lies flat, 19 mm tall. A straight approach to a
19 mm object probably brings the fingertips or fingers into the table. The
planner's shallow depth exists for this case but did not help.

## 13. Hammer

**Shape.** 126 × 335 × 33 mm, 0.26 kg. Moments of inertia about the CoM are
about 3 × 10⁻³ kg·m² about both axes across the handle; the CoM is at the mesh
bounding-box centre. Held across about 35 mm at 114-116 N per finger. No gs
data.

**Runs.** 2, both lost in the arm swing with the base still.

**s7 (lost at step 493).**
- Retreat: 2.5 mm.
- On the very first swing step (403 → 404), arm_qvel jumped from 0.003 to
  0.214. The swing starts at full rate with no ramp. Both finger forces dropped
  from 115 to **29-36 N for two steps**, then recovered.
- The hammer then slid +13.5 mm toward the fingertips (up) and -11.6 mm
  sideways during the swing.
- At step 492 the force fell to 95 N and contact was gone at 493.

**s42 (lost at step 687).**
- Almost no slip through the swing: 1-2 mm until step 655.
- Then the forces became asymmetric, 146 vs 108 N, while dx went -5 (down) and
  dz +5. That is a growing load along the closing axis.
- The arm stopped at step 685. On step 686 the **wrist-roll alignment** started
  (arm_qvel 0.002 → 1.73). Forces fell to 98/73 N and then to 3 N on step 687,
  and contact was gone at 690.

**Why (Inference).** The hammer combines low grip (115 N, narrow) with a long,
heavy-inertia body.

- **The roll.** It turns the gripper about its own axis at 0.1 rad per step,
  giving a peak angular kick of about 1,580 rad/s².
  - If the handle is across the roll axis, the inertia torque is about
    3 × 10⁻³ × 1,580 ≈ 4.7 N·m.
  - The friction couple that resists it is about μNw = 0.25 × 115 × 0.035 ≈
    1.0 N·m.
  - So the roll alone can tear it out, as seen in s42. The hammer's orientation
    in the grasp is not logged, so this depends on that assumption.
- **The swing start.** The unramped start of the swing caused the s7 force
  collapse. The following slide then fits the swing kicks on a 0.26 kg object at
  115 N (u_n ≈ 0.53, the highest of all).

## 14. Foam brick

**Shape.** 52 × 78 × 51 mm, 0.185 kg.

**Grip.** It depends on which faces are grasped: 158-169 N across the 51 mm
side, 209-274 N across the 78 mm side. t s42, c17 s42 ×2 and c17 s7 got
250-274 N; t s7 got 167 N.

**Runs.** 5, 3 delivered.
- Delivered: t s42, t s7, c17 s7.
- Lost: c17 s42 twice, at steps 721 and 1142. Both losses are seed 42 runs, and
  the t s42 run on the same seed was delivered.

**Contact geometry (Data, gs).**
- **Area contact**: up to 8 points per pad, spans 8-46 mm along x and 10-22 mm
  across.
- CoM 7-17 mm toward the fingertips from the contact centre.
- Hold slip ≤ 2.2 mm.
- All 6 gs runs completed the swing with 2-6 mm slip.

**Slip (Data).**
- **Delivered runs.** Retreat ≤ 1.8 mm. Swing 2.6-4.0 mm. Driving 3-8 mm at the
  end. The t s42 reach then added +21 mm dx (toward the fingertips) in state 68,
  but it was still delivered.
- **c17 s42 (lost at step 721).** It moved -6 mm in dz in the retreat and -24 mm
  more in the swing, reaching (-8, -2, -30) mm before the base moved. It held at
  that offset through 70 steps of driving and was lost in a corner.
- **c17 s42 (lost at step 1142).** It moved +9 mm toward the fingertips in the
  retreat, then +8.5 dx and +15 dz in the swing, reaching (18, -1, 15) mm. It
  crept to dz +30 over 470 driving steps and was lost while cruising.

**Why (Inference).** This is the best-conditioned object.

- Flat patch contact gives large torsional radius. Mass relative to grip is
  moderate: u_n ≈ 0.17 at 250 N.
- Corner torsion demand is about 0.2 N·m against about 1.6 N·m of capacity.
- When the slip stays at 3-4 mm through the swing, it is delivered. Both losses
  began with an unusually large slip in the retreat and swing, 23-32 mm, which
  left the pads near the edge of a 51-78 mm face.
- Why those two runs slipped early, at full 250 N grip, is **not explained by
  the data**. It was not low force, and their gs-style geometry is not logged.

---

## Patterns across objects

1. **Grip force is set by width, not by need.** With the fingers commanded
   shut, force is 6.6 N per mm of opening: 115-145 N for the banana, hammer and
   strawberry, 300 N for the sugar box. The narrow objects that are also long or
   heavy (banana, hammer) get the least margin.

2. **The kick along travel is a normal force, but it still unloads the trailing
   finger.** At 30 mm per step it removes about m × 470 / (2N) of the grip each
   step:

   | Object | u_n |
   |---|---|
   | hammer | 0.53 |
   | sugar box | 0.50 |
   | potted meat can | 0.47 |
   | soup can | 0.34 |
   | banana | 0.32 |
   | apple | 0.24 |
   | foam brick | 0.17-0.26 |
   | strawberry | 0.08 |

   The three highest are all lost. This is the strongest single
   predictor I found. It is not sufficient on its own: the soup can is
   delivered 4 of 4. (Inference)

3. **Most damage is done before driving.** The retreat and the arm swing (base
   still) cause the first, and often decisive, slip:
   - 4 of the 13 traced losses (sugar box s42, hammer ×2, banana s7) happen
     before the base drives.
   - Two more (potted meat, foam brick c17 721) come within 30-70 steps of
     driving, after 25-32 mm of swing slip.
   - In the delivered runs the swing slip is 1-12 mm. (Data)

4. **The wrist-roll alignment is a hazard.** It turns at 0.1 rad per step at the
   end of every swing and in corners. It is harmless (< 1-2 mm) for most objects,
   but it immediately broke the hammer s42 grasp and added 13 mm to the potted
   meat slip. Its angular kick (about 1,580 rad/s² peak) matters for long,
   high-inertia objects. (Data for timing; Inference for mechanism)

5. **Losses happen at a pad boundary.** Typical slip is a slow creep of 0.01-0.3
   mm per step, followed by loss within 1-5 steps once the contact reaches:
   - the 12 mm pad edge (apple, strawberry), or
   - the fingertip end (strawberry c17 s42, banana in the retreat), or
   - the edge of a flat face (sugar box, potted meat, foam brick at 30-85 mm).

   Forces stay steady until 1-3 steps before. Grip force is therefore not an
   early warning; accumulated drift is. (Data)

6. **Point contacts creep even when nothing moves.** The strawberry, and some
   apple grasps, slid 2-9 mm in the stationary hold. Area or line contacts (foam
   brick, soup can) did not. (Data)

7. **Corners matter more than straights for sideways (dz) slip.** Turning steps
   carry most of the dz change for the soup can, strawberry and sugar box, and
   correlate with the estimated lateral kick for the soup can. The soup can's
   23 mm "slip" is most likely a pivot about the closing axis, driven by its CoM
   sitting 20-48 mm above the contact. (Data / Inference)

8. **Run-to-run variation is large.** The same object and seed swap between
   delivered and lost:
   - banana s7 and s42 swap between the t and c17 batches;
   - foam brick s42 was delivered once and lost twice;
   - potted meat s7 "did not rise" once and lifted normally on rerun;
   - master chef can s42 failed at step 68 once and reached planning on rerun.

   One or two runs per object are not enough to give rates.

9. **Seven objects never reach the carry.** Their failure is upstream of
   slippage:
   - no antipodal candidates: bowl, plate;
   - no plan, IK and approach failures: master chef can, mustard bottle, large
     marker;
   - object on the floor: mug;
   - navigation never finishes: bleach cleanser.
