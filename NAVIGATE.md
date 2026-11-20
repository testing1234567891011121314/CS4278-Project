# NAVIGATE.md: carrying the object to the goal

What the carry (states NAVIGATE and REACH, plus the hold taken before them)
has to cope with, every attempt at it so far, what each attempt failed on,
what was changed, and what that fixed or broke. The pick itself is in
`GRASP.md`; changes 21-25 there are the first carry attempts and are repeated
here in more detail. Change numbers continue from `GRASP.md`.

**Evidence level.** Every full run is one trial per object on maze seed 42,
cloud seed 300 (`tools/pick_all.py --deliver`, output in `runs/deliver_NN/`).
One trial is not a success rate: an object that is delivered once may fail on
another cloud draw. Single-object experiments are named where they are used
(`tools/experiments/side_hold.py`, `trace_carry.py`; logs in `runs/side_hold/`).

## 1. What the env does to a carried object

These are measured, not assumed.

| Fact | Size | Consequence |
|---|---|---|
| The base is position-controlled: each physics step it covers 30 % of the gap to its target, and the target moves once per tick (5 physics steps). | The base's speed jumps at every tick: about 15,780 x step m/s2 for one physics step, so 158 m/s2 at 10 mm per tick. | The object follows that jump only through the pads. This "jolt" is what shakes objects loose, and it grows in proportion to speed. |
| Pad friction 0.25, no spinning friction. | Turning an object about the line between the pads is resisted by about 0.1 N m on the hammer's handle. | **The pads' line is a loose hinge.** The object settles with its centre of mass below that line whatever the hand does (measured: hand turned 90 degrees, hammer followed 31). |
| The fingers squeeze by position control. | About 6,000 N per metre the finger target lies inside the object: 110-120 N on a 30 mm handle, 13 N on the plate's rim, which a fully closed finger cannot squeeze further. | Thin objects are held weakly however hard the hand closes. |
| When a jolt is larger than the pads' friction, the contact slides for that instant. | Hammer: jolt force 100-120 N against 44-58 N of friction. | While it slides sideways, nothing holds the weight: the object creeps **down** through the pads a little every tick (hammer: 1 mm per 40 ticks). |
| The maze has 1 m cells; the base is 0.6 m across. | In the pose the arm lifted the object in, the elbow is 25-35 cm to one side of the base's centre line. | The robot with its arm out is wider than the base; at corners the elbow or wrist comes within millimetres of a wall. |
| The route is 21 m; the lift ends at tick 160-500. | About 2,400 ticks are left. | The object has to move at 9-12 mm per tick on average; there is no time for a second carry. |

## 2. Factors in the carry

| Factor | Where | Now | Effect of changing it |
|---|---|---|---|
| Speed of the object | `Config.carry_haste`, `carry_reserve` | 1.25 x what the ticks left require: 9-12 mm per tick | 30 mm per tick lost nine objects (attempt 1); pacing recovered three (attempt 2). Slower is safer but the budget does not allow it. |
| Who leads | `navigation/carry.py` | The object is led along the route; the base trails an arm's length behind, heading toward the object | The arm stays inside every corner. Not varied. |
| Turn on the spot before the drive | `CarryFollower`, phase `turn` | The object goes round at the carry speed | A long object held level lags this turn and yaws in the hand (hammer: 15-19 degrees). |
| Wrist roll during the drive | `CarryPlan.roll`, `Config.aligned_share` | With the hand pointing down, the pads' line is kept along the travel | A pad then pushes the object at each jolt, instead of friction holding it. Only possible with the hand upright. |
| **Hold for the carry** | `Solver._plan_hold`, `Config.carry_hold`, `level_pads`, `hang_lever`, `askew`, `hold_reach` | Pads' line rolled level; hand laid level where the object hangs askew; arm folded into the plane of the base's centre line | New in attempt 5. See section 4. |
| **Which grasp, for the carry's sake** | `Solver._hangs`, `Config.hang_long`, `hang_grip`; `rank_by_mass` | A long object lying down is gripped between its centre of mass and its nearer end | Hammer: from never delivered to delivered (section 4). |
| Distance kept from walls | `Config.carry_clearances`, `carry_margin` | Routes at 0.42, 0.38, 0.34 m from walls; every pose checked at 12 mm | Tighter margins let routes through but the real arm touched; looser ones refuse the bowl and plate. The fold makes the robot narrower instead. |
| Torso | `_plan_carry`, `rise` | Raised toward the goal's height before the drive | Shortens the final reach. |
| Final reach | `Solver._reach` | Straight line that puts the object, not the hand, on the goal | Change 24 fixed a reach from a fully stretched arm. |

## 3. All attempts side by side

Tick of delivery, or how the trial ended. "lost" = both pads off the object
during the carry; "no route" = every route failed the collision check;
"n cm" = distance from the goal when the trial ended.

| Object | 1: 30 mm/tick | 2: paced | 3: longer budget | 4: mass ranking | 5: side hold | 6: fold fallback | 7: mass ranking off | 8: full squeeze | 9: slower torso | 10: changes 35-37 |
|---|---|---|---|---|---|---|---|---|---|---|
| master chef can | lost | lost | lost | lost | lost | lost | lost | lost | lost | lost |
| sugar box | lost | lost | lost | lost, 185 cm | lost, 62 cm | lost, 62 cm | **2,431** | lost | lost | lost, 268 cm |
| tomato soup can | **1,242** | **2,679** | **2,410** | **2,406** | **2,419** | **2,419** | **2,419** | **2,436** | **2,436** | **2,401** |
| mustard bottle | **1,582** | **2,453** | **2,453** | **2,446** | **2,457** | **2,457** | **2,457** | **2,482** | **2,482** | **2,643** (re-picked) |
| potted meat can | lost, 54 cm | **2,595** | **2,376** | **2,375** | **2,388** | **2,388** | **2,388** | **2,407** | **2,407** | **2,367** |
| banana | lost, 56 cm | **2,684** | **2,414** | **2,415** | **2,424** | **2,424** | **2,421** | **2,452** | **2,452** | **2,401** |
| strawberry | lost | lost, 55 cm | **2,406** | **2,680** (re-picked once) | **2,416** | **2,416** | **2,699** (re-picked twice) | **2,442** | **2,442** | lost, 51 cm |
| apple | **1,274** | **2,645** | **2,416** | lost | lost | lost | lost | lost | **2,454** | **2,411** |
| bleach cleanser | lost | lost | lost | lost | lost | lost | lost | lost | lost | lost |
| bowl | no route | no route | no route | no route | no route (**2,341** alone, after change 31) | **2,332** | **2,346** | **2,345** | **2,345** | **2,321** |
| mug | lost | lost, 52 cm | lost, 50 cm | out of ticks in the reach, 27 cm | **2,505** (re-picked once) | **2,505** | lost, 53 cm, then no route | lost | **2,706** (re-picked) | **2,501** (re-picked) |
| plate | no route | no route | no route | no route | lost after 6 m | lost after 6 m | lost after 6 m | lost | lost | lost |
| large marker | lost | 59 cm, goal out of reach | 59 cm, goal out of reach | lifted at 780, lost | **2,740** (lifted at 780, past the 600 deadline) | **2,740** (lifted at 780) | **2,755** (lifted at 757) | **2,810** (lifted at 780) | **2,810** (lifted at 780) | **2,676** (lifted at 780) |
| hammer | lost | lost, 48 cm | lost | lost | **2,354** | **2,354** | **2,354** | **2,384** | **2,384** | lost |
| foam brick | **1,238** | **2,681** | **2,417** | **2,414** | **2,424** | **2,424** | **2,432** | **2,449** | **2,449** | **2,401** |
| **Delivered** | **4** | **6** | **7** | **6** | **9** | **10** | **10** | **9** | **11** | **9** |

Runs: `runs/deliver_01` to `runs/deliver_10`. Attempts 1-4 predate this file
and were reconstructed from those result tables and from `GRASP.md` section 6.

## 4. Each attempt: what failed, what was changed, what that did

### Attempt 1: first carry, object led at up to 30 mm per tick (change 21)

- **Design.** Arm kept as it was after the lift; object led along a route with
  rounded corners; base trailing; wrist roll steering; whole carry checked in
  the planning copy; straight reach at the end.
- **Result.** 4 of 15 (soup can, mustard, apple, foam brick), at ticks
  1,238-1,582.
- **Failures.** Nine objects lost. The potted meat can slid down out of the
  fingers once the speed passed about 14 mm per tick; the banana slipped in
  the corners. Both got within 56 cm of the goal. Bowl and plate: no route.
- **Cause.** The jolt (section 1) is proportional to speed. At 30 mm per tick
  it is three times what the pads' friction holds for most objects.

### Attempt 2: speed paced to the ticks left (change 22)

- **Change.** The object is led only 1.25 times faster than the remaining
  ticks require: 9-11 mm per tick instead of 30.
- **Fixed.** Potted meat can and banana delivered. 6 of 15.
- **Broke.** Deliveries moved from tick ~1,250 to ~2,650. Five of the six ran
  into the NAVIGATE state's own budget (2,000 ticks) and had to re-plan with
  the object in hand, which cost them about 250 ticks each.
- **Still failing.** Strawberry lost 55 cm short; mug, hammer and marker got
  within 48-59 cm but lost the object or could not reach the goal.

### Attempt 3: NAVIGATE budget 2,000 to 2,900 ticks

- **Change.** The state budget no longer cuts a carry that is on schedule.
- **Fixed.** No more re-plans: the six deliveries moved to ticks 2,376-2,453,
  and the strawberry was delivered. 7 of 15, the best full run before the
  side hold.
- **Still failing.** Master chef can, sugar box, bleach cleanser and hammer
  lost; bowl and plate no route; mug lost 50 cm short; marker 59 cm short with
  the goal out of the arm's reach.

### Attempt 4: grasps ranked by the centre of mass (changes 23, 24, 25)

- **Changes.** (23) Grasps whose pads' line passes beside the centre of mass
  are ranked down, on the reasoning that the weight would swing the object.
  (24) The arm solver leaves out a joint stuck on its limit. (25) Squeeze 30
  to 20 mm, tested and not kept. Collision margin for the carry set to 12 mm.
- **Fixed.** (24) The mug's final reach, straight up from a stretched arm, no
  longer stops at "out of reach": the mug got to 27 cm before the ticks ran out.
- **Broke.** (23) delivered nothing new and cost the apple (lost in the
  carry) and the marker (first lift at tick 780, past the 600 deadline). 6 of 15.
- **What this attempt got wrong.** It treated swinging as a fault to avoid.
  The hammer was already gripped 14 mm from its centre of mass and was still
  lost, so swinging under gravity was not its problem (see below).

### Between attempts 4 and 5: what the single-object tests showed

The user's suggestion: hold the hammer from the side, on the z axis, with the
head pushing it down into the hold; the same for the plate; and make this the
hold for navigation, since that is where objects are lost.

| Test | What was seen | What it means |
|---|---|---|
| Hammer, old hold, traced every 10 ticks | During the base's turn on the spot the hammer yawed 15-19 degrees about the fingers' axis, then worked loose at 2.1 m. About the pads' line it moved only 5 degrees. It touched nothing. | The long handle, held level, lags every turn of the base. Not a gravity swing. |
| Hand turned +90 degrees about the pads' line | Hammer followed for 31 degrees, then fell back to hang with its centre of mass below the line. | The pads' line is a hinge: the hand cannot stand the hammer up, only gravity or a stop can. |
| Hand turned -66 degrees (as far as the arm reached) | Hammer followed (the palm pushed it) and was carried 15 m instead of 2 m before it was lost. | Standing the handle up is worth a factor of seven in distance. |
| Plate, hand rolled 90 degrees | Plate followed and hung on edge, still held. Its route was then refused: wrist 12 mm from a wall. | Rolling works; the refusal is about the arm, not the plate. |
| Which link is nearest at each refused pose | Elbow, forearm or wrist, 0-12 mm from a wall, at one or two corners. | With the arm as it lifted the object the robot is too wide for 1 m cells. |
| Arm spread in the post-lift pose | Elbow 25-35 cm to the side of the base's centre line. | The hold must fold the arm into the plane of the centre line. |

### Attempt 5: the side hold (changes 26-30)

Built in four steps, each tested on the hammer and the plate.

| Step | Change | Hammer | Plate | Side effect |
|---|---|---|---|---|
| a | Hold v1: roll the pads' line level; lay the hand on its side when the centre of mass is more than 4 cm from the line; pull the hand in 0.7 m from the base. Hammer gripped at least 6 cm from its centre of mass (either side). | Picked and held. Route refused: elbow 8-9 mm from a wall. | Route refused: wrist 9 mm from a wall. | **Soup can broken**: it was laid over although it already hung straight down, and its route was refused. |
| b | Hold v2: the arm is folded into the vertical plane through the base's centre line (spread 22-26 cm down to 12-16 cm). The hand is laid level only where the object hangs more than 20 degrees askew. | Hung on the z axis within 5 degrees. Carried 9.3 m, then lost: it spun 87 degrees about its own axis onto the handle's narrow side (grip 115 to 89 N) and crept down through the pads 1 mm per 40 ticks until it dropped out. | Fold refused: the arm passed 4 mm from the robot's own body. | Soup can no longer laid over. |
| c | Grasp on the **head side** of the centre of mass (between the centre of mass and the nearer end), at least 3.5 cm from it. The head is then above the fingers and the long handle hangs below. Arm-against-own-body check relaxed to "not inside" (the env does not collide the robot with itself). | **Delivered at tick 2,392.** In the hold it turned 20 degrees in 21 m and then stopped moving: the head had come to rest on the fingers. | Lost while folding: where the arm solver missed a step the path fell back to a plain joint blend, and the hand swung down into the base. | |
| d | The fold continues from each step to the next and is refused rather than blended if the arm cannot follow; a second way in (turn in place, then fold) if the first is blocked; up to six folded poses tried. | **Delivered at tick 2,361.** | Hold taken; route clear for the first time; plate hangs on edge. Lost after 6 m: its rim is gripped with 13 N per finger. | |

Why the hold works, in terms of section 1:

- **On the z axis** the base's turns spin the hammer about its own handle,
  which has almost no inertia, instead of swinging 33 cm of hammer round.
- **Centre of mass below a level pads' line** is the one pose gravity does not
  pull the hinge away from.
- **Head above the fingers** turns the downward creep from a loss into a
  stop: the hammer slides until the head sits on the fingers and stays there.
- **Arm folded in the centre plane** makes the robot as wide as its base, so
  the corners that refused the bowl, plate and hammer are passable.

Full run of all 15 with the side hold: section 5.

## 5. Result of attempt 5

One run of all 15, maze seed 42, cloud seed 300 (`runs/deliver_05`), with
changes 26-30: **9 of 15 delivered**, at ticks 2,354-2,740. The six that were
delivered in attempt 4 or 3 and are again (soup can, mustard, potted meat can,
banana, strawberry, foam brick) arrive within 15 ticks of where they did
before: folding the arm did not disturb them.

| Object | Outcome | What the hold did | Cause where it failed |
|---|---|---|---|
| hammer | delivered 2,354, first carry | laid level, folded | |
| mug | delivered 2,505 | folded | lost once in the carry and picked up again from the floor; before the fold it ran out of ticks |
| large marker | delivered 2,740 | folded | its first lift is at tick 780, past the 600 deadline, so this trial would still lose the lift points (change 23, see `GRASP.md`) |
| bowl | no route | tried to lay the hand over (the bowl hangs 30 degrees askew from its rim grasp), found no way, kept the wide pose | fixed by change 31: delivered at 2,341 in a single run |
| plate | lost after 6 m | rolled on edge, folded | 13 N per finger on the rim |
| sugar box | lost 62 cm from the goal, then no route for the second carry | folded | carried 20 m of 21 before it was lost: the farthest yet |
| master chef can | lost | folded | the grasp is tilted 30 degrees and off the pad middle, as before |
| bleach cleanser | lost | folded | 1.13 kg: the jolt is about as large as the grip |
| apple | lost | folded | as in attempt 4 (change 23); delivered in attempts 1-3 |

### Attempt 6: fold without laying when laying is blocked (change 31)

The bowl's hold asked for the hand to be laid over, the arm could not follow
that turn, and the wide post-lift pose was kept. Now the arm is folded with
the hand as it is whenever laying fails. Bowl alone: spread 30 cm to 6 cm,
route clear, **delivered at tick 2,341**.

Full run (`runs/deliver_06`): **10 of 15 delivered**. The bowl arrives at
tick 2,332 on its first carry; the nine deliveries of attempt 5 repeat at the
same ticks; the five failures (master chef can, sugar box, apple, bleach
cleanser, plate) fail as in attempt 5.

### Attempt 7: the same code with the mass ranking switched off

`--set rank_by_mass=False` (`runs/deliver_07`); nothing else differs from
attempt 6. This tests the claim made after attempt 4 that change 23 cost the
apple and the marker's lift time.

- **Result.** 10 of 15 again, but a different ten.
- **Gained.** Sugar box delivered at tick 2,431, on its first carry: its
  first delivery in any run.
- **Lost.** Mug: lost 53 cm from the goal, then no route for a second carry
  (delivered at 2,505 in attempt 6 after one re-pick).
- **Worse.** Strawberry needed two re-picks and arrived at 2,699 instead of 2,416.
- **Not changed.** Apple still lost in the carry with the same lift tick
  (216); the marker's first lift is at tick 757 instead of 780, still past
  the deadline. **So the switch does not bring back what attempt 4 lost, and
  the claim that change 23 alone cost them is not supported.** (The switch
  restores the score and removes the "swings" tier; it does not restore the
  order of tight fits or the lever used by the lift rule, which changed in
  the same step.)
- **Read.** With one trial per object the two settings are level. Eleven
  different objects have now been delivered across attempts 6 and 7; never
  more than ten in one run.

### Attempts 8 and 9: fuller squeeze, slower torso (changes 32-34)

Target set by the user: 15 of 15, a re-pick only where there is no choice.
The five failures of attempt 6 were traced one at a time first.

| Object | What the trace showed |
|---|---|
| apple | Held at 195 N per finger with under 11 mm of slip for 1,000 ticks, then gone in 10 ticks, at the same corner in every run. It is gripped 4 mm from the palm and turns about the pads' line; when a bulge reaches the palm it is levered out. |
| master chef can, mug | Lost in the turn on the spot that starts the carry. Both are gripped with the hand tilted 30-54 degrees, so the wrist's roll cannot steer the pads' line. |
| sugar box | Gripped near its middle it is a see-saw: it turns about the pads' line at 1.3 degrees per tick from the first jolt after the lift and is levered out by the palm at 20 degrees, with the grip force unchanged at 192 N. Its gripped faces are parallel (2 degrees, measured in the planning copy), so it is not squeezed out. |
| bleach cleanser | Its first pick fails ("object moved during the drive"); picked up again lying on the floor it slides 17 mm down the fingers during the turn on the spot. |
| plate | 13 N per finger on the rim; lost after 6 m (unchanged). |

Every object gripped with the hand pointing straight down was delivered on
its first carry in attempt 6; every failure but the plate had a tilted hand or
a centre of mass beside the pads' line.

**Correction.** That pattern was read off the last grasp of each object in
one run and does not hold over all carries. Counting every carry in attempts
5, 6, 7 and 9 that ended either delivered or lost (72 carries, many of them
the same carry repeated on the same seeds):

| Hold | Delivered | Lost | Lost objects |
|---|---|---|---|
| hand straight down | 31 | 21 | apple, bleach cleanser, mug, strawberry, sugar box |
| hand tilted | 4 | 6 | master chef can, sugar box |
| side hold (laid or rolled) | 5 | 5 | bleach cleanser, plate |

A straight hand is delivered 60 % of the time and a tilted one 40 %: a weak
tendency, not a rule. What the traces do support, object by object, is the
centre of mass lying beside the pads' line (sugar box), the object turning
into the palm (apple), and creep along the fingers (sugar box, hammer).

| # | Change | Full-run result | Kept? |
|---|---|---|---|
| 32 | In the hold a hand pointing mostly down is stood upright, the object tilting with it, so the roll can steer (`stand_up`). | Single runs: can, sugar box, mug and apple lost sooner or at the same place. The tilted object's weight then turns it about the pads' line. | No (`stand_up` 2.0 = never) |
| 33 | Fingers commanded fully closed for the carry (`carry_squeeze`): 190 N becomes 290 N on the sugar box. | Attempt 8 (`runs/deliver_08`): **9 of 15**. Apple, mug and sugar box lost; nothing gained. | No |
| 34 | Torso rise 4 to 2 mm per tick (each tick's rise is a vertical jolt of 6 g at 4 mm). | Attempt 9 (`runs/deliver_09`, with 32 and 33 off): **11 of 15**. **Apple delivered** at 2,454 on its first carry; mug delivered again after a re-pick; all others as in attempt 6, about 20 ticks later. | Yes |

### After attempt 9: the heavy objects (changes 35-37, single-object tests)

| # | Change | What it did |
|---|---|---|
| 35 | The torso creeps up 0.3 mm per tick all through the carry instead of in a phase of its own; the route is checked at both torso heights. | No separate vertical jolt; saves about 50 ticks. The sugar box was still lost: it is not the torso that turns it. |
| 36 | The long-object rule (change 27) now covers the sugar box: at least 17 cm long, 1.7 times as long as wide, and at least 0.3 kg (which leaves the 66 g banana alone). | Sugar box gripped 65-74 mm from its centre of mass: hangs on end straight below the hand, hold turns under 10 degrees, carried **18.7 m of 21.4** (was lost in the first 20 ticks). |
| 37 | Where the weight hangs more than 6 cm below the pads the wrist keeps the pads' line across the travel (`swing_lever`), so the object swings about the line instead of being rocked out along it. | Applied to the hanging sugar box. Not yet shown to matter on its own. |

**Creep.** The hanging sugar box is lost in the end because it slides down
the fingers about 2 mm per metre carried and runs out of the pads through
its top end after 33-41 mm, 2.5 m short of the goal. Measured against that:

| Tried | Creep | Outcome |
|---|---|---|
| carry at 10 instead of 11.6 mm per tick | 1.6 instead of 2.3 mm per 100 ticks, about the same per metre | lost 2.5 m short, at the same place |
| full squeeze, 290 N | faster: 14.5 mm in the first 300 ticks against 11.9 | lost at tick 1,321 |
| half squeeze, 97 N | faster: 15.1 mm in the first 300 ticks | lost at tick 1,335 |

So the creep is not a shortage of grip. It is what the hammer's head resting
on the fingers stops, and the sugar box has nothing of the kind.

### Attempt 10: changes 35-37 on all 15, and their withdrawal

Full run with changes 35, 36 and 37 as defaults (`runs/deliver_10`):
**9 of 15**, two fewer than attempt 9.

- **Lost that attempt 9 delivered.** Strawberry (lost in the carry, then
  picked up again four times, ends 51 cm short) and hammer (lost in every
  carry). Neither is touched by changes 36 or 37, which leaves change 35:
  with the torso rising all through the drive there is a small vertical jolt
  on every tick of the carry instead of on 50 ticks before it.
- **Not gained.** Sugar box: hangs and is carried, but creeps out 268 cm
  short, as in the single runs.
- **Worse.** Mustard bottle needed four re-picks (2,643) instead of one.
- **Unchanged.** The other eight deliveries, 35-50 ticks earlier.

**Decision.** The defaults are back to those of attempt 9. Changes 35-37
stay in the code as settings that are off: `CarrySettings.torso_step`
(0.002, in a phase of its own), `Config.long_length` 0.25, `long_ratio` 2.0,
`long_mass` 0, `swing_lever` 1.0 (never). `runs/deliver_11` repeats the 15
on these defaults: **11 of 15, each delivery at the same tick as in attempt
9** (soup can 2,436, mustard 2,482, potted meat can 2,407, banana 2,452,
strawberry 2,442, apple 2,454, bowl 2,345, mug 2,706 after a re-pick, marker
2,810 with its lift at 780, hammer 2,384, foam brick 2,449). Not delivered:
master chef can, sugar box, bleach cleanser, plate. This is the committed state.

### The full YCB library as a second test set

At the user's request the library `github.com/kwonathan/ycb_urdfs` is a git
submodule at `assets/ycb_urdfs` (78 objects, 15 of them the course's).
`tools/ycb_library.py` writes the other 63 to `assets/ycb_extra/` in the
course's format, and `tools/pick_all.py --library assets/ycb_extra` runs
them like the course's.

The library's own URDFs could not be used: meshes scaled by 10, every mass
1 kg, unit inertia. What the conversion assumes, checked against the
course's own 15 with `tools/ycb_library.py --check`:

- **Mass and inertia** from the volume of the convex pieces at 900 kg/m3.
  For the course's eleven solid objects this is within 12 % of the course's
  mass. For the thin-walled ones it is too heavy, because the library's
  pieces are thicker than the course's: bowl 0.11 kg against 0.033, plate
  0.20 against 0.058, mug 0.20 against 0.14.
- **Collision shape** is the library's V-HACD decomposition; the course
  uses CoACD. Contacts will differ in detail.
- **Contact parameters** are the URDF defaults, as for the course's objects.

So a library object is a plausible object of the same kind, not a copy of
what a grader would load. First run: `runs/library_01`.

### What the library run shows: where the problems are

`runs/library_01`: the 63 further objects, on the committed code (`f09377d`),
the env's own maze, cloud seed 300. Tables: `tools/carry_report.py
runs/library_01` (saved as `runs/library_01/report.txt`; the same tool on the
course's `runs/deliver_11` is in `runs/deliver_11/report.txt`). One trial per
object: these are counts that show where to look, not rates.

**How far the 63 got**

| Stage | Objects |
|---|---|
| reached the grasp | 59 |
| lifted 10 cm | 53 |
| held 50 ticks | 48 |
| reached NAVIGATE | 53 |
| reached REACH | 26 |
| delivered | 24 |

The pick loses 15 of 63 and the carry loses 27 of the 53 that get to it.
The carry is the larger blocker.

**1. Most objects are lost at the start of the carry, not along the way.**
Of 63 carries that lost the object, 38 lost it within 400 ticks of entering
NAVIGATE, 30 of them between ticks 200 and 400. Those ticks are the hold, the
torso's rise, the wrist rolls, the turn on the spot and the first metre of
the drive. The course's own run shows the same: 8 of its 10 losses fall in
the first 400 ticks. The logs do not say which of those steps it is; the
travel-pose survey records it per pose.

| Ticks after entering NAVIGATE | Carries lost |
|---|---|
| 0-199 | 8 |
| 200-399 | 30 |
| 400-999 | 17 |
| 1,000 on | 8 |

**2. Speed.** The later a carry starts, the faster it must go, and the less
often it arrives.

| Carry's speed | Carries | Delivered |
|---|---|---|
| 10-12 mm per tick | 37 | 13 |
| 12-16 | 19 | 5 |
| 16-22 | 16 | 4 |
| over 22 | 17 | 2 |

Carries that start after tick 1,500 deliver 4 of 23. A re-pick that late is
close to worthless as things stand.

**3. Mass.** No object over 0.6 kg was delivered (0 of 10 carries; 2 of 7
between 0.2 and 0.6 kg; 16 of 42 between 0.05 and 0.2 kg). Very light ones do
not do better (6 of 30 under 0.05 kg): they are the small and thin ones.

**4. How full the hand is.** Grasps wider than 85 mm, close to the hand's
103 mm, deliver 2 of 15, and are lost soonest (median 272 ticks).

**5. Depth in the hand.** Contacts 0-8 mm from the back of the pads deliver
17 of 45; contacts 12-24 mm out deliver 7 of 44. This fits the creep along
the fingers seen on the sugar box: a deep grasp has more pad left to slide on.

**6. What does not separate delivered from lost.** The hand's direction
(straight down 18 of 70, tilted 6 of 17), wrist steering (18 of 72 against 6
of 17), and the size of the turn on the spot (11 of 44 under 45 degrees, 13
of 45 above). The side hold was taken three times in the whole run (laid
twice, rolled once) and delivered none.

**7. Recovery.** After 61 drops that were followed by another attempt, the
object was picked up and lost again 21 times, picked up and delivered or
still being carried at the end 13 times, and not picked up 27 times (lost in
the lift 6, no grasp plan 5, lost in the back-off 4, and others).

**8. The pick's own blockers** (10 objects never lifted, 16 attempts ended
"object moved during the drive").

| Blocker | Objects |
|---|---|
| wider than the open hand everywhere | pitcher base, mini soccer ball, large wood block, chain (lying flat) |
| flat and thin on the table: raked, then no candidates | fork, spoon, knife, nine-hole peg board |
| too small to hold | dice |
| arm does not arrive at the approach or transit, eight times | Rubik's cube |
| round, still rolling when the base drives up ("object moved during the drive") | lemon, orange, plum, softball, tennis ball, marbles; the same message is the first failure of the course's bleach cleanser, mustard bottle and mug |

**What this means for the travel-pose trials**

- Record where in the set-up the object is lost (taking the pose, the turn,
  the first metre). That is where three losses in five happen.
- Test each pose at more than one speed: speed matters more here than the
  hand's direction does.
- Include the heavy and the hand-filling objects on purpose: nothing over
  0.6 kg has been delivered by any pose so far.
- Record slip along the fingers per metre: depth in the hand is the one
  grasp property that separates delivered from lost, and it cannot be
  changed after the lift, so the pose has to slow the creep instead.

## 6. Change log (continues `GRASP.md` section 6)

| # | Change | Fixed | Broke or left failing |
|---|---|---|---|
| 21-25 | See `GRASP.md` section 6 and attempts 1-4 above. | | |
| 26 | `Config.rank_by_mass` switches change 23 on or off (default on, as before). | Lets the two rankings be compared on the same draw. | On cloud 300 the hammer's first grasp is the same either way. |
| 27 | A long object lying down (at least 25 cm, twice as long as wide) is gripped across its length between the centre of mass and the nearer end (`_hangs`, `hang_long`, `hang_grip` 3.5 cm). These grasps go first and replace the "swings" tier for such objects. | Hammer delivered (2,392 and 2,361 in two single runs). | With the grip on the far side of the centre of mass (first version, 6 cm either side) the hammer crept out after 9.3 m. |
| 28 | Hold for the carry (`_plan_hold`): roll the pads' line level, lay the hand level where the object hangs askew, fold the arm into the plane of the base's centre line. `Config.carry_hold` switches it off. | Hammer delivered; mug and marker delivered (full run 5); plate gets a clear route. | First version laid the soup can over and its route was refused; fixed by the "askew" condition. Plate still lost after 6 m. |
| 29 | The way into the hold continues step to step; no fall-back to a joint blend; turn-then-fold as the second way; six end poses tried. | Plate no longer thrown into the base while folding. | |
| 30 | A refused carry reports the base pose and the name of the nearest robot link (`CollisionChecker.nearest`). | Showed that every refusal was the elbow, forearm or wrist. | |
| 31 | If the hand cannot be laid level, the arm is folded anyway with the hand as it is. | Bowl delivered (2,341, single run): first time. | |

## 7. Open problems

*(Rewritten after attempt 9; the first four items below are current, the rest are as first written.)*

- **Sugar box.** Needs either a grasp so near its centre of mass that it does
  not see-saw (delivered once that way, attempt 7), or 40 mm more room for
  the creep when it hangs.
- **Master chef can.** Only a tilted grasp is within reach from where the base
  parks. Standing the hand up afterwards does not help; the grasp itself has
  to be upright, which is a question of where the base parks.
- **Bleach cleanser.** The first pick fails before any carry; the carry of
  the re-picked bottle has not been studied on its own.
- **Creep in general.** Every heavy object slides along the fingers at about
  2 mm per metre. A 21 m route costs about 40 mm of the 58 mm pad.

- **Plate.** 13 N per finger on the rim; hanging on edge puts its whole
  weight on friction. It may be better held flat with the lower finger under
  it and only the arm folded.
- **Master chef can, sugar box, bleach cleanser.** Heavy objects gripped at
  the centre of mass: the jolt at 9-12 mm per tick is about as large as the
  grip. Not yet traced with the fold in place.
- **Speed.** Every delivery arrives after tick 2,350. A dropped object cannot
  be picked up and carried again in the ticks left.
- **Marker.** Its landing after it is thrown out of the table is chaotic
  (`GRASP.md` section 5); with change 23 on, its lift is past the deadline.
- **Change 23** (mass ranking) is still on. Switching it off (attempt 7)
  trades the mug for the sugar box and does not bring back the apple or the
  marker's lift time, so what broke those two in attempt 4 is not yet known.
- **Apple.** Lost in the carry in attempts 4-7 after three deliveries; not
  yet traced with the fold in place.
- **Sugar box and mug.** Each is delivered under one ranking and lost under
  the other, 53-62 cm from the goal: both are close to the limit of the hold
  at this speed.
