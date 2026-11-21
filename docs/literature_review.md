# Literature review: picking, carrying and delivering an object with a mobile manipulator

Written 2026-11-21. A synthesis of three reviews in this repository, set against our own measurements:

| Document | Covers | Sources |
|---|---|---|
| `old/paper_review/lift_torque_literature.md` (with `lift_torque_model.md`, `FInal_synthesis.md`) | Grasp wrench quality, torsional friction and the limit surface, pivoting in a parallel jaw, grasp choice by the centre of mass, how Bullet represents a contact patch; Mavrakis et al. on post-grasp effort | 35, most read in full |
| `docs/lit_transport_holds.md` | Holds that survive transport; caging and basket holds for small or round objects | 10; two read in full, the rest from abstracts |
| `docs/lit_cited_references.md` | The six references of the report: RRT-Connect, A*, force closure, GPD, YCB, PyBullet | 6; four read in full, Nguyen from abstracts, PyBullet from its guide and source |

Our measurements: `NAVIGATE_V2.md` sections 7.23 and 7.24 (benchmark 1: 1,500 trials, 15 objects on 100
mazes), `GRASP.md` for the pick.

How to read the tags: **[P]** a source says it; **[D]** a derivation by us or by a reviewing agent, not a
published result; **[H]** a hypothesis not yet tested; **[C]** a fact about our code or data. The
derivations and the figures taken from our asset files in the two new reviews were made by agents and
have not been re-checked by a second reading; section 9 lists what is unverified.

## 1. The task, and where each part of the literature meets it

A Fetch-like robot picks one of 15 objects from a point cloud, carries it about 22 m through a maze and
places it within 10 cm of a goal, in 3,000 control ticks, in PyBullet.

| Part of the task | Literature we stand on | State |
|---|---|---|
| Route of the base | A* (Hart, Nilsson, Raphael 1968) | Settled. Not where points are lost |
| Transit of the arm | RRT-Connect (Kuffner, LaValle 2000) | Settled. Not where points are lost |
| Grasp candidates from a cloud | Force closure for two contacts (Nguyen 1988); detection without a model (ten Pas et al. 2017) | Works for 12 objects; three are hard |
| Grasp quality | Ferrari-Canny, Dex-Net 1.0 to 4.0, distance of the contacts' centre from the centre of mass (Roa, Suárez 2015) | Partly used; torsion is weaker in the simulator than in the models |
| Holding the object while it is carried | Balance of the gravito-inertial wrench (Heins, Schoellig 2023); critical speed for contact stability (Pham, Pham 2019) | **Open. 72 % of our lost points** |
| Holds that do not rely on friction | Energy-bounded and basket caging (Mahler et al. 2016, 2018; Rimon et al.; Rodriguez et al. 2012) | Planar theory only; nothing for a moving hand |
| The objects and the simulator | YCB (Calli et al. 2015); PyBullet (Coumans, Bai) | Our masses and hulls differ from YCB; the simulator's position control is the source of the jolt |

## 2. Planning: settled, and not the problem

- **A\*.** [P] With a heuristic that never overestimates, A* returns an optimal path, and with a
  consistent one no comparable algorithm expands fewer nodes. [C] Our base route is A* on an 8-connected
  grid with the octile heuristic, which is admissible and consistent, on obstacles inflated by a
  clearance; the path is then shortened and its corners rounded, so the route hugs the inflated walls.
- **RRT-Connect.** [P] Two trees, one from each end, each greedily extended toward the other;
  probabilistically complete, no claim of optimality, three to four times faster than a single tree in
  open space. [C] Our arm transit is this, with the straight move tried first and a budget counted in
  work steps.
- **Evidence that neither is the problem.** [C] In the logs of benchmark 1 the three hard objects fail
  at the grasp pose (no arm pose, or the pose in collision: 12,849 of 13,088 candidates for the can), not
  in the transit (32). When the object stays in the hand the robot arrives; the one planning failure is
  a carry refused for want of clearance (56 of 1,500 trials, 29 of them the plate).
- **One usable idea [D].** A* stays optimal for any step cost at least the step's length, so a cost
  that grows near walls keeps the guarantee and runs the route down the middle of corridors. It lengthens
  the route, which raises the speed the ticks force, so it has a price (section 4).

## 3. Grasp synthesis: what the theory explains about the three hard objects

- **Force closure.** [P, abstracts only] Nguyen: a two-contact grasp with friction is force-closure when
  the line joining the contacts lies inside both friction cones. [C] Our antipodal candidates apply this
  with a 15° cone; with the simulator's effective friction of about 0.25 the cone is 14°. The thin-wall
  candidates (rims) do not apply it.
- **The plate [D].** Its rim is held with the pads about 45° off the line between the contacts
  (`GRASP.md` section 5), which is not force-closure at any friction we have. That is the analytic reason
  the hold survives only while the hand moves straight up. The plate is delivered in 24 trials of 100.
- **Detection from a cloud.** [P] ten Pas et al. detect grasps without a model, label them as
  frictionless antipodal grasps allowing about 1 mm of error, and report 93 % success in clutter; "the
  object dropping out after an initially successful grasp" is listed as a failure class and not analysed.
  [C] We enumerate opposed point pairs and check the open hand against the collision shape; we have no
  learned classifier.
- **The can [C].** The cloud's mesh is 102.5 mm across, the collision hull 103.8 mm, the open hand
  102.85 mm. Every collision hull we measured is 0.6 to 3 mm larger than the mesh the cloud is sampled
  from. [H] "No grasp plan" (228 times in benchmark 1) is then a margin of about a millimetre, not a
  failure of the search.
- **Quality by the centre of mass.** [P] The contacts' distance from the centre of mass is the one
  published quality measure with direct evidence for transport: Kolamuri et al. found that regrasping
  toward the centre of mass stops rotation in the hand while more grip force "does little". Mavrakis et
  al. choose grasps by the arm's effort after the grasp but assume the object never moves in the hand,
  which is exactly what fails for us.
- **The objects are not YCB's [C].** Our masses differ from the published ones by up to a factor of
  more than six (hammer 259 g against 665 g, foam brick 185 g against 28 g, can 1,054 g against 414 g). No physical
  number from YCB may be used as a prior; the solver rightly reads the simulator's.

## 4. Holding the object in transit: the centre of the problem

**What the simulator does [P for the control law, D for the consequences].**

- PyBullet's position control is a velocity constraint: each physics step sets the joint's speed to the
  gain times the remaining error over the time step. The environment commands a new target once per tick
  and runs five physics steps, with gain 0.3.
- So within every tick the speed falls as 1, 0.7, 0.49, 0.34, 0.24 of its peak, and jumps back at the
  next tick. As an acceleration over one physics step the jump is 15,800 s⁻² times the step per tick:
  202 m/s² (21 g) at the median carry speed of 12.8 mm per tick, 126 m/s² at 8 mm per tick, which is
  about the slowest the ticks allow.
- Every joint follows the same law at the same instants, so no mix of base, torso and arm changes the
  ratio of jolt to speed. The two levers are the hand's mean speed and the direction in which the jolt
  meets the pads.
- Friction can hold at most 2μN/m across the closing line: 141 m/s² for the sugar box and 85 m/s² for
  the can at 180 N per pad. At route speeds an object held by friction alone must therefore slip a little
  at every tick.
- The simulator has no torsional friction at a contact and at most four contact points per pair, so a
  hold resists twisting less than a real flat pad would (`lift_torque_literature.md` sections 6 and 7).
  Changing contact or dynamics parameters is ruled out for this project.

**What the literature offers.**

- **The balance law** [P, Heins and Schoellig]: the contact forces must cancel the object's
  gravito-inertial wrench, m(a − g) and the terms from the centre of mass, inside the friction cone.
  With no friction the support's normal must lie along a − g: the waiter's tilt. Planning with the
  smallest friction that is feasible adds robustness.
- **The limit is on acceleration, not speed** [P, Pham and Pham]: contact stability is linear in the
  acceleration along the path; at 120 % of the critical speed the object already slips about a
  centimetre.
- **Friction weakens under vibration** [P, abstracts; Popov et al.]: oscillation in the contact plane
  lowers the effective static friction. It is the nearest real-world relative of our creep.
- **Grasps stable at rest fail under the task's dynamic wrench** [P, abstract; Trupin et al.]: a grasp
  should be scored by the wrench along the motion, not at the lift.
- **Security comes from the palm** [P, abstracts; Cutkosky]: enveloping grasps with palm contact are
  what people choose when the hold must not fail.

**How this reads against benchmark 1.**

- [C] The solver's friction-based speed limit does not predict a drop (mustard bottle, limit 3.7 mm per
  tick, drops in 6 trials of 100; strawberry, limit 20, in 64). [D] That is what the jolt analysis
  expects: at route speeds every friction-held object slips each tick, so what decides the outcome is
  not whether it slips but where the slips add up to, and whether something stops them.
- [C] The traces agree: hand level, the bleach creeps into the palm and stops there; hand down, the
  creep runs toward the fingertips, the way out; the can sinks across the pads only in bends and half a
  turn of the wrist puts it back above the axis.
- [D] So the travel pose matters through the direction of the net drift and through what lies
  downstream of it (palm, lower finger, or nothing), not through a friction margin.
- [C] Two kinds of loss: heavy or weakly held objects go in the drive at any distance; small ones
  (strawberry, mug) go while the hand is being re-oriented or in the first half metre. The second kind
  is the subject of section 5.

**What the literature does not give.** Nothing on a jaw gripper carrying an object on a base that jolts
every tick; nothing on how deep in the jaw to hold an object for transport; no model of creep driven by
a simulator's impulses. Every friction-based limit therefore has to be calibrated from measured creep.

## 5. Holds that do not rely on friction

- **Energy-bounded caging** [P, abstracts; Mahler et al.]: under gravity a cage need not be complete; it
  is enough that the object can only leave by gaining energy, and the escape energy measures the hold.
  Planar only.
- **Basket grasps** [P, abstracts; Rimon, Pokorny, Wan; Shirizly et al.]: two fingers can hold an object
  against gravity with no squeezing; the security is the depth of the basket. Planar only.
- **From caging to grasping** [P, abstract; Rodriguez, Mason, Ferry]: with two fingers every cage can be
  closed into a grasp without breaking it. Our use is the converse: a pinch that loses friction falls
  back to a cage only if a cage existed around it.
- **Objects with holes** [P, abstract; Pokorny, Stork, Kragic]: a handle can be held topologically, by
  closing a loop through it. The mug has one.
- **What follows for us [D].**
  - Two flat pads cannot cage a round object: the strawberry can leave within the pads' plane without
    rising, so a pinch of it is held by friction alone in every orientation. A cage needs a third
    surface, the palm or the lower finger as a floor. "Level, finger under" is a basket; "hand down" is
    the opposite.
  - No source checks a cage while the hand is re-oriented, and that is where we lose the strawberry and
    the mug. The sound test is the depth of the support under the object along the whole way into the
    pose, not only at its end.
  - A thin cylinder pinched across its axis escapes along the axis when that is level. The marker hung
    straight down is the pose in which neither gravity nor a fore-aft jolt loads that direction.

## 6. What the literature does not cover

These are the gaps, and so the places where our measurements are the contribution:

1. Loss of a grasped object from a parallel jaw during long transport on a mobile base. Grasp papers
   stop at the lift; non-prehensile transport papers carry on a tray.
2. Creep under periodic impulses well inside the Coulomb limit, as a simulator produces it.
3. Three-dimensional caging with flat pads, and caging along a re-orientation of the hand.
4. The depth of the grasp in the jaw as a variable for transport.
5. Choosing the travel pose after the grasp is fixed: the literature chooses the grasp for the task; we
   are given the grasp and choose the pose.

## 7. Hypotheses and tests, in the order we would take them

Usable with the grasp frozen (travel pose and control only):

| # | Hypothesis or rule | From | Test |
|---|---|---|---|
| 1 | The net drift must point at the palm or a finger, never at the open side of the jaw | Section 4; Cutkosky | Per pose, the direction of the measured creep in the hand frame; benchmark 2 gives the loss rate of every pose for every object |
| 2 | A hold whose 2μN/m is above the jolt at the slowest pace should be led at that pace, with no haste | Jolt law; Pham and Pham | Already in the code for the weakest holds (V2-28); its ablation in benchmark 2 measures it |
| 3 | The small objects are lost where the support stops being under them on the way into the pose | Basket depth | Log the support's depth under the object every tick while the pose is taken (no change of behaviour) |
| 4 | With the centre of mass straight below the closing axis and the axis along the travel, neither gravity nor the jolt twists the object | Balance law | Pads along against pads across, same seeded draws |
| 5 | Banking the hand into bends by atan(v²κ/g) removes the steady sideways load | Heins and Schoellig | Bleach and sugar box, finger under, with and without |
| 6 | A route down the middle of corridors is refused less often | A* with a clearance cost | Plate and can, count the refused carries and the route's length |

Diagnosis in our own planning copy of the simulator, which is ours to change: creep per metre against
speed for each object and pose, with the base driven at a constant step; the same with the base still;
and the ratio of friction used to friction available at the pads, logged per tick.

Only if the grasp limits are loosened: hold the plate so that it hangs below the pads; prefer the grasp
nearest the centre of mass for the bleach and the can and seat it against the palm; apply the friction
cone to rim grasps; hold the mug through its handle.

**One correction between the two reviews.** `lit_transport_holds.md` proposes limiting the change of the
base's step per tick (rule R3). By the jolt law in `lit_cited_references.md` the jolt inside a tick
depends on the step itself, not on its change, so such a limit can only help where the carry speeds up
or brakes. The lever is the mean speed.

## 8. The six references of the report

| Reference | Citation as given | What we use of it |
|---|---|---|
| Kuffner, LaValle, "RRT-Connect", ICRA 2000, pp. 995-1001 | Correct (pages from index records) | The arm's transit planner |
| Hart, Nilsson, Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", IEEE Trans. SSC 4(2):100-107, 1968 | Correct, as printed | The base's route |
| Nguyen, "Constructing Force-Closure Grasps", IJRR 7(3):3-16, 1988 | Agrees with index records; full text not read | The two-contact test behind the antipodal candidates |
| ten Pas, Gualtieri, Saenko, Platt, "Grasp Pose Detection in Point Clouds", IJRR 36(13-14):1455-1473, 2017 | Correct; the DOI resolves | Grasping from the cloud without a model; we do not use its classifier |
| Calli et al., "Benchmarking in Manipulation Research: Using the Yale-CMU-Berkeley Object and Model Set", IEEE RAM 22(3):36-52, 2015 | Correct; issue number not seen in print | The object set; our masses and hulls differ from it |
| Coumans, Bai, "PyBullet", pybullet.org, 2016-2021 | Matches the authors' own BibTeX | The simulator; its position control explains the jolt |

Do not confuse the YCB magazine article with arXiv 1502.03143, which has a different title.

## 9. What is not verified

- Read from abstracts or index records only: Nguyen 1988, all the caging papers, Cutkosky, Popov,
  Trupin. Two details unconfirmed: the venue and year of the Rimon, Pokorny, Wan paper, and the author
  list of the 2016 energy-bounded caging paper.
- The jolt law's numbers (15,800 s⁻² per metre of step, the 0.7 profile) are derived from the documented
  control law with the environment's gain; whether the server overrides the motor's error-reduction
  parameter was not checked. The same factor appears independently in `FInal_synthesis.md`, and it has
  not been measured in a trace.
- The masses, mesh sizes and hull sizes in section 3 were read from the asset files by an agent and not
  re-measured.
- Everything tagged [D] or [H] is ours and untested. In particular the reading in section 4, that drift
  direction and a downstream stop decide a carry, fits the traces and benchmark 1 but has not been
  tested as a prediction; benchmark 2 records every carry's pose and outcome and can test it.
