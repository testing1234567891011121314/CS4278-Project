# The six cited references: what they say, what we use, what to test

Written 2026-11-21. Read-only review: no code changed, no simulation run. It does not repeat
`old/paper_review/lift_torque_literature.md` (torsion, limit surfaces, the Bullet manifold), NAVIGATE_V2.md
7.23-7.24 or GRASP.md, and assumes the reader knows them.

Tags: **[S]** read in the source itself; **[M]** metadata, an abstract or an index record only; **[U]** not
verified; **[D]** my arithmetic or reasoning; **[C]** a fact about our code or data, with the file named;
**[H]** a hypothesis to test.

## 0. Summary

| # | Reference | Bibliography as given | Read | Bears on the open problems? |
|---|---|---|---|---|
| 1 | Kuffner & LaValle, RRT-Connect | Correct (pages from index records) | full text | No: our transit planner hardly ever fails |
| 2 | Hart, Nilsson & Raphael, A* | Correct, printed on the pages | full text | A little: route clearance |
| 3 | Nguyen, force-closure grasps | Agrees with index records; paper not read | abstract and the companion thesis abstract | Yes: the plate's rim pinch is not force-closure |
| 4 | ten Pas et al., GPD | Correct; the DOI resolves | full text (arXiv) | A little: robustness to 1 mm of error |
| 5 | Calli et al., YCB | Correct; title as printed | full text | Yes: our assets are not the YCB masses, and the can's hull is wider than the hand |
| 6 | Coumans & Bai, PyBullet | Matches the authors' own BibTeX | Quickstart Guide plus two source files | Yes: the env's position control makes a jolt at every tick that is fixed by the speed alone |

## 1. Kuffner & LaValle, RRT-Connect (ICRA 2000)

**(a) Claims [S].**
- Two RRTs, from start and goal. Each iteration EXTENDs one tree by a step ε toward a random sample;
  CONNECT repeats EXTEND on the other tree toward the new vertex until Reached or Trapped; then SWAP
  (Fig. 5). Voronoi-proportional selection drives exploration (Fig. 4).
- Corollary 1: probabilistically complete; vertices converge to the sampling distribution. No convergence
  rate, no optimality claim.
- CONNECT is "often by a factor of three or four" faster in open spaces; "pathological cases exist".

**(b) Bibliography.** Title, authors and venue are correct; the PDF header says "In Proc. 2000 IEEE Int'l
Conf. on Robotics and Automation (ICRA 2000)" [S]. The copy has no page numbers, so pp. 995-1001 comes from
index records [M]. DOI 10.1109/ROBOT.2000.844730 resolves, but I did not confirm it is this paper [U].

**(c) Our code [C].** `grasping/arm_planner.py: plan_transit` is RRT-Connect as in Fig. 5, for the arm's
transit only, with a linear nearest-neighbour scan as in the paper. Where we differ: the straight joint
move is tried first; ε = 0.30 in a plain Euclidean metric mixing torso metres and arm radians (the paper
assumes a metric ρ, says nothing of units); every edge is checked at 0.06 steps, not just the new
configuration; the budget is work steps (`tree_work` 12,000), not iterations K; 40 random shortcuts.

**(d) Use.** Failure reasons summed over the `runs/bench_01/logs` of failed or dropped trials [C]:

| Object | Candidates tried | Transit failures | No arm pose | Pose blocked |
|---|---|---|---|---|
| Master chef can | 13,088 | 32 | 5,707 | 7,142 |
| Plate | 825 | 1 | 349 | 73 |
| Large marker | 260 | 0 | 56 | 62 |

The pick fails at the grasp pose (IK or collision), not in RRT-Connect, so nothing here is worth a day.

## 2. Hart, Nilsson & Raphael, A* (1968)

**(a) Claims [S].** Nodes are expanded in order of f(n) = g(n) + ĥ(n). Theorem 1: if ĥ(n) ≤ h(n) for all n,
A* is admissible (it finds an optimal path). Under the consistency assumption, eq. (5),
h(m, n) + ĥ(n) ≥ ĥ(m), Theorems 2 and 3 show that no admissible algorithm "no more informed" than A*
expands fewer nodes (Theorem 3 covers ties).

**(b) Bibliography [S].** The header reads "IEEE Transactions of Systems Science and Cybernetics, vol.
SSC-4, no. 2, July 1968", and the pages are numbered 100 to 107. DOI 10.1109/TSSC.1968.300136 resolves.

**(c) Our code [C].**
- `navigation/a_star.py: find_path` is A* on an 8-connected grid (costs 1 and √2) with the octile
  heuristic, exact on an empty grid, so admissible and consistent [D]. Stale heap entries are skipped and
  corner cutting is forbidden.
- Beyond the paper: a binary grid inflated by a clearance (`transport.carry_grids`), then `shorten` and
  `round_corners`. So the route runs along the inflated walls.

**(d) Use.** In 7.23, 56 trials stopped "with no drop", mostly for "no clear route" (plate 29, can 11),
where the rim grasp makes the arm stick out sideways. Theorem 1 holds for any edge cost at least the step
length, so a clearance cost (for example +k/d_wall per cell) keeps ĥ admissible and consistent [D] and
centres the route in corridors. The price is a longer route, so a faster carry and a harder jolt
(section 6). One-day test: the cost on the `carry_grids` routes only; plate and can on the same seeds as a
baseline; count "no clear route" stops and route length.

## 3. Nguyen, "Constructing Force-Closure Grasps" (IJRR 1988)

**(a) Claims.**
- Abstract, via index records [M]: fast algorithms build force-closure grasps from the object's shape, with
  an independent region of contact per finger; most non-marginal equilibrium grasps are force-closure.
- Thesis (MIT AITR-905, 1986) abstract [M]: a non-marginal equilibrium grasp is force-closure with two
  frictional point contacts in 2D, or two soft-finger or three hard-finger contacts in 3D.
- The full text was blocked (403), so equation-level content is [U]. The two-contact test the thesis
  restates: the line joining the contacts must lie inside both friction cones, of half-angle atan μ.

**(b) Bibliography [M].** IJRR 7(3):3-16, June 1988, DOI 10.1177/027836498800700301 (it resolves). This
agrees with the index records; I did not see it printed.

**(c) Our code [C].**
- `grasping/candidates.py: antipodal_pairs` is the two-contact test, with `cone` = 15°. With the review's
  μ ≈ 0.25, atan μ = 14.0° [D], so pairs leaning 14-15° are just outside the friction cone (minor).
- `thin_walls` skips the cone: it accepts the far contact wherever n_i·n_j < -0.7, up to about 46° from
  antiparallel [D]. No independent regions: a candidate is two points; the pads' area is checked only for
  collision.

**(d) Use.**
- **Plate.** GRASP.md section 5 found the rim pinch is "a jam", with the pads at 45° on a wedge rim. A pair
  45° off the line is not force-closure for μ = 0.25 [D], which is why it holds only while the hand moves
  straight up. The grasp library is frozen, so this explains rather than changes. If limits are loosened
  later: require |n_j·line| ≥ cos(atan μ) for thin walls and count the plate candidates left.
- **Independent regions** suggest requiring the cone for every cloud point under each pad: flat facets,
  more manifold spread, more torsion via the review's r_eff [H]. It changes ranking, so it waits too.
- **Can and marker:** nothing here; they fail on reach and collision (section 1) and ejection (section 6).

## 4. ten Pas, Gualtieri, Saenko & Platt, "Grasp Pose Detection in Point Clouds" (IJRR 2017)

**(a) Claims [S, arXiv 1706.09911 full text].**
- Grasps come from the cloud with no segmentation or pose estimate.
- Candidates (Sec. 4, Alg. 2): a Darboux frame at a sampled point (v3 smoothed normal, v2 major curvature
  axis), a grid of y offsets and z rotations (|Y| = 10, |Φ| = 8), each hand pushed to contact and kept if
  it misses the cloud with points between the fingers. A 4-layer CNN classifies them (15 image channels).
- Labels (Sec. 6.1) are "frictionless antipodal" on the full mesh, with about 1 mm of vertex error
  allowed; such grasps are force-closure for any non-zero friction. I found no cone angle [U].
- Results: 93 % success in dense clutter with active sensing, 84 % passive, 75 % without the selection
  heuristic, 53 % without the classifier; top grasps beat side grasps (Sec. 8.5). "The object dropping out
  after an initially successful grasp" is a failure class (Sec. 8.6) that is not analysed.

**(b) Bibliography.** Title and authors are correct [S]. 36(13-14):1455-1473, 2017 comes from a BibTeX
record and the British Library catalogue [M]. The DOI resolves to the SAGE page, which I could not read
(403) [M].

**(c) Our code [C].**
- Like GPD, `antipodal_candidates` uses only the cloud and normals, and is deterministic. Unlike GPD it
  enumerates opposed point pairs (no Darboux grid, no classifier) and checks the open hand against the
  collision shape in the planning copy (`hands_that_fit`), where GPD checks against the cloud. Its normals
  are the visual mesh's face normals, from the env (`pick_nav_reach_env.py:124`).
- Our "from above" weight of 0.25 agrees with GPD's top-grasp finding.

**(d) Use.**
GPD's 1 mm perturbation bears on the **can**, which gets only "tight" fits: the cloud (visual mesh) is
102.5 mm across, the collision hull (`coacd_allinone.obj`) 103.8 mm, the open hand 102.85 mm [C].
One-day check, no grasp change: re-plan the logged can candidates jittered by ±1 mm in the planning copy.
A low share still collision-free and reachable means "no grasp plan" (228 times in 7.23) is a margin
problem, not a search problem [H].

## 5. Calli et al., "Benchmarking in Manipulation Research" (IEEE RAM 2015)

**(a) Claims [S].** The YCB set: 77 entries in Table 2, in five classes, each with RGB-D scans, textured
meshes and "major dimensions and mass" "to enable realistic simulations". Watertight meshes come from
Poisson reconstruction (it fails on transparent or reflective parts); URDF and KinBody files are given. A
protocol template and five protocols (pitcher-mug, gripper assessment, table setting, block pick-and-place,
peg insertion). Friction and inertia data are left to future work.

**(b) Bibliography [S].** The printed title is "Benchmarking in Manipulation Research: Using the
Yale-CMU-Berkeley Object and Model Set", by Calli, Walsman, Singh, Srinivasa, Abbeel and Dollar. It was
published 11 Sept. 2015, starting on p. 36, DOI 10.1109/MRA.2015.2448951. Vol. 22 and pp. 36-52 come from
the CMU record, and "no. 3" was not seen printed [M]. arXiv 1502.03143 has a different title; do not mix
the two.

**(c) Our data [C].** The readme says the objects are "adapted", and they differ from Table 2. URDF mass
against YCB, in grams:

| Object | Ours | YCB | Object | Ours | YCB | Object | Ours | YCB |
|---|---|---|---|---|---|---|---|---|
| master chef can | 1,054 | 414 | banana | 172 | 66 | mug | 138 | 118 |
| sugar box | 638 | 514 | strawberry | 48 | 18 | plate | 58 | 279 |
| tomato soup can | 319 | 349 | apple | 247 | 68 | large marker | 27 | 15.8 |
| mustard bottle | 612 | 603 | bleach cleanser | 972 | 1,131 | hammer | 259 | 665 |
| potted meat can | 354 | 370 | bowl | 33 | 147 | foam brick | 185 | 28 |

- Ours look like volume times one density [H]; the solver reads the simulator's mass (`_grip`), rightly.
- Every hull checked is 0.6-3 mm larger than the visual mesh the cloud comes from (visual against
  collision, mm): can 102.5 × 140.2 against 103.8 × 141.1; plate 260 × 26.7 against 261.6 × 29.1; marker
  19.2 × 120.9 against 20.3 × 121.9.

**(d) Use.** Take no physical prior from YCB. Treat the cloud's width plus 1-2 mm as the real width for
clearance; that bounds the plate's 1-4 mm flange overlaps (GRASP.md section 2) and the can's tight fit.
The gripper protocol's note that flat objects need nail-like fingertips matches the plate needing a finger
under it.

## 6. Coumans & Bai, PyBullet

**(b) Bibliography [S].** The bullet3 README's BibTeX: Erwin Coumans and Yunfei Bai, "PyBullet, a Python
module for physics simulation for games, robotics and machine learning", http://pybullet.org, 2016--2021.
The given citation matches. The Quickstart Guide we read is dated 2016-2022.

**(a) What the documentation says that bears on a held object slipping [S].**
- **POSITION_CONTROL is a velocity constraint**: "the final target velocity is computed using:
  kp*(erp*(desiredPosition-currentPosition)/dt)+currentVelocity+kd*(m_desiredVelocity - currentVelocity)",
  and `force` is "the maximum motor force used to reach the target value". Source defaults: m_erp = 1,
  m_kd = 1 (`btMultiBodyJointMotor.cpp`); unspecified gains in `setJointMotorControlArray` are kp 0.1,
  kd 1.0 (`pybullet.c`). Whether the server overrides m_erp is [U].
- **Contact forces** from `getContactPoints` are those "applied during the last stepSimulation" (substep).
- **Friction drift.** `changeDynamics(frictionAnchor)` is "friction drift correction (disabled by default,
  unless set in the URDF contact section)"; `frictionERP` applies only with anchors on.
- **Solver.** `numSolverIterations` is a maximum, and `solverResidualThreshold` (1e-7) can stop it earlier.
  `enableConeFriction` is on by default; turning it off is "worth trying ... in case there are friction
  artifacts". AABBs are extended by `contactBreakingThreshold` (0.02 m). `useSplitImpulse` limits "huge
  penetration recovery forces", in maximal coordinates only. Link damping 0.04; `maxJointVelocity` 100.

**(c) How the env and our code use it [C].**
`pick_nav_reach_env.py: step` drives every joint by POSITION_CONTROL: `forces` 2,000,000 (which overrides
the URDF finger `effort="60"`), kp 0.3 (fingers 0.03), kd at its default of 1, 5 substeps of 1/240 s per
tick. The base is three joints (x, y, yaw) of a fixed-base robot. No URDF has a `<contact>` section, so
there are no friction anchors. `solver.py: _carry_step` already models the base jolt with this law
(`base_gain` 0.3, erp taken as 1).

**Consequences [D]** (with kd = 1 and a target velocity of 0, each substep sets q̇ = kp·e/dt):
- **The error shrinks by 0.7 each substep**: speed runs 1, 0.7, 0.49, 0.34, 0.24 of peak, then jumps back.
- **The jolt is fixed by the speed.** The jump at a tick boundary, as an acceleration over one substep, is
  15,800 s⁻² times the step per tick:

  | Step per tick | Acceleration at each tick | Peak (mean) speed |
  |---|---|---|
  | 12.8 mm (7.24's median) | 202 m/s², about 21 g | 1.11 (0.61) m/s |
  | 8 mm (about the slowest the ticks allow) | 126 m/s² | |
  | 3 mm | 47 m/s² | |

- **No command sequence avoids it.** Every joint resets at the same boundary with the same 0.7 profile,
  so base, torso and arm share one within-tick shape, and any mix moves the hand with the same
  jolt-to-speed ratio. The only levers are the hand's mean speed and the direction the jolt meets the
  pads.
- **Friction alone cannot hold most objects.** Across the closing line friction holds at most 2μN/m: 141
  m/s² for the sugar box (N = 180 N, μ = 0.25), 85 m/s² for the can. So a friction-only hold must slip a
  little every tick at route speed; along the closing line the pad pushes, and a finger under bears load.
- Corners (1.2 m/s² at 0.6 m/s on a 0.3 m fillet) and gravity (9.8 m/s²) are small by comparison.
- **[H] Creep is a per-tick ratchet**: slip at each boundary, catch-up in the deceleration, and any
  asymmetry (gravity, a turn, the yaw) leaves a net drift. It is not a drop predictor: 7.24 found that the
  solver's friction limit does not predict drops (the 48 g strawberry drops often).

**(d) Tests for one day that leave the env's physics alone.** The lift-torque review records that changing
dynamics parameters is ruled out; the readme does not say so itself [U]. The planning copy
(`planning_sim.py`, a second DIRECT client) is ours, so it can be used for diagnosis.
1. **Creep rig.** In the planning copy, with the env's gains and substeps, hold each object in its travel
   pose and drive the base at 3, 8 and 13 mm/tick, along and across the closing line; log the object's pose
   in the hand per substep. Repeat with `frictionAnchor` on the pads: if the creep stops, the cause is
   velocity-level drift. Output: creep (mm per metre) against speed per pose, for the carry's pace to use
   in place of the static limit.
2. **Static hold.** The same rig, base still, 2,000 ticks. Creep without motion is solver drift under
   gravity, not the jolt [H].
3. **Slip margin monitor (cheap).** Each tick, from `getContactPoints` on the pads, u = |(f₁, f₂)|/(μ·f_n)
   per pad; log it in `tools/trace_trial.py` for the sugar box, hammer, strawberry and mug. If u nears 1
   before each loss, slow the drive where u > 0.8 (Costanzo's k_s, applied to speed). Caveat: u is sampled
   on the last and gentlest substep.
4. **Marker ejection [H].** The hand joins the marker's contact pairs and solver island within 2 cm, before
   any touch. Log `getClosestPoints(hand, marker)` and the marker's speed per tick in a trace. If the
   ejection always starts within about 2 cm, the trigger is the robot joining the island of an object in
   penetration recovery. That fits the landing varying with `transit_step` (GRASP.md section 5), and
   favours an approach that stays 2 cm off until the close.

## 7. Sources

- RRT-Connect: https://ics-websites.science.uu.nl/docs/vakken/mcrws/papers_new/Kuffner_LaValle%20-%202000%20-%20RRT-Connect.pdf ; https://researchr.org/publication/KuffnerL00%3A0
- A*: https://people.stfx.ca/jdelamer/courses/csci-564/_downloads/b2220c66675ddde471ca1795147b8e86/A_Formal_Basis_for_the_Heuristic_Determination_of_Minimum_Cost_Paths.pdf ; https://dblp.org/rec/journals/tssc/HartNR68
- Nguyen: https://scilit.net/publications/7084ef2181158a2b852e1666ff5c6901 (search snippet only); thesis https://dspace.mit.edu/handle/1721.1/6849
- GPD: https://arxiv.org/abs/1706.09911 (via ar5iv); https://rhys-newbury.github.io/bibliography/ten2017grasp.html
- YCB: https://www.eng.yale.edu/grablab/pubs/Calli_RAM2015.pdf ; https://ri.cmu.edu/?p=6007
- PyBullet: https://raw.githubusercontent.com/bulletphysics/bullet3/master/docs/pybullet_quickstartguide.pdf ; bullet3 master `README.md`, `src/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp`, `examples/pybullet/pybullet.c`
- All DOIs checked to resolve at doi.org on 2026-11-21; landing pages not read.
