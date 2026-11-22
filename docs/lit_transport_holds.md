# Literature: holds that survive transport, and caging holds for small or round objects

Written 2026-11-21, about an hour of reading; read-only on the code, no runs. Does not repeat
`old/paper_review/lift_torque_literature.md` (Q_DCC, Q_MNF, Dex-Net gravity wrench, Kolamuri regrasp
toward the CoM, torsional friction, PyBullet manifold) or `old/paper_review/FInal_synthesis.md`
(Mavrakis et al., post-grasp effort). Own data: `NAVIGATE_V2.md` 7.4 to 7.6, 7.23, 7.24.

How far each source was checked:

- **[read]**: the method and results sections were read through the arXiv HTML.
- **[abs]**: only the abstract, a publisher record or search snippets. No PDF-to-text tool was
  available here, so these claims are as the abstract states them.
- **[mem]**: background knowledge, not checked against the source today.
- **[D]**: my own derivation from the sources and our data, not a published result.

Standing constraint (memory): the grasp library is frozen, and a carry problem must not be fixed by
changing the grasp. The rules are therefore split into **travel-pose / control rules** (usable now)
and **grasp rules** (only if the grasp limits are loosened later).

---

## 1. Choosing a hold that survives being carried

### Sources

| # | Source | Checked |
|---|---|---|
| T1 | A. Heins, A. P. Schoellig, "Keep it Upright: Model Predictive Control for Nonprehensile Object Transportation with Obstacle Avoidance on a Mobile Manipulator", IEEE RA-L 2023, doi:10.1109/LRA.2023.3324520; https://arxiv.org/abs/2305.17484 | [read] Sec. IV to VII |
| T2 | H. Pham, Q.-C. Pham, "Critically fast pick-and-place with suction cups", ICRA 2019 [venue: mem]; https://arxiv.org/abs/1809.03151 | [read] Sec. II to IV |
| T3 | N. Trupin, Z. Wang, A. H. Qureshi, "Physics-Conditioned Grasping for Stable Tool Use", arXiv 2025 (under review); https://arxiv.org/abs/2505.01399 | [abs] |
| T4 | M. R. Cutkosky, "On grasp choice, grasp models, and the design of hands for manufacturing tasks", IEEE T-RA 5(3):269–279, 1989, doi:10.1109/70.34763; and M. R. Cutkosky, R. D. Howe, "Human grasp choice and robotic grasp analysis", in *Dextrous Robot Hands*, Springer 1990, https://biorobotics.harvard.edu/pubs/1990/human%20grasp%20choice.pdf | [abs] |
| T5 | Reduction of friction by vibration: V. L. Popov, Frontiers in Mechanical Engineering 6:69, 2020, doi:10.3389/fmech.2020.00069; M. Popov, V. L. Popov, N. V. Popov, Friction 5(1), 2017, doi:10.1007/s40544-016-0136-4 | [abs] (exact titles not checked; see the doi) |

Also relevant, and already covered in `lift_torque_literature.md` section 5: S29 Mavrakis 2016,
S31 Roa & Suárez (Q_DCC), S32 Kolamuri 2021, S5 Dex-Net 4.0.

### What each one actually claims

**T1 (Heins & Schoellig).**

- The object must not move relative to the end effector. Its gravito-inertial wrench is
  m(v̇ₑ − Rₑg) plus the CoM terms m(ω̇ₑ× + ωₑ×ωₑ×)c, plus the torque Jω̇ₑ + ωₑ×Jωₑ (Eq. 2). The
  contact forces must balance it inside a friction pyramid, ‖fᵗ‖₁ ≤ μfⁿ (Eqs. 1, 3).
- No desired orientation is given. The constraints tilt the tray as needed, and the plotted runs
  tilt during motion (Sec. VI, VII-A).
- With zero friction (Sec. V), the tangential part of v̇ₑ − Rₑg + (CoM terms) must vanish. In
  words, the support normal lies along the gravito-inertial acceleration.
- Planning with the smallest friction coefficient that is feasible when static adds robustness.
  Validated on a velocity-controlled mobile manipulator at up to 2.0 m/s and 7.9 m/s².
- It is a tray, not a pinch, but the constraint is the one our pads must meet.

**T2 (Pham & Pham).**

- The contact-stability constraint is linear in the joint acceleration, plus a term in the velocity
  squared (Eq. 10). Gravity enters through the Newton–Euler balance (Eq. 9).
- Retiming without the constraint failed in 16 of 20 paths. Paths run at 120 % and 140 % of the
  critical speed slipped "nearly 1 cm and 10 deg" (Sec. IV-B).
- No explicit safety margin is described.
- Lesson: the limit belongs on acceleration along the path, per object and hold, not on speed.

**T3 (Trupin et al.).** Grasps that are stable quasi-statically fail under the dynamic task wrench
("lever-arm amplification"). Choosing the grasp that minimises the predicted wrench along the
trajectory gave 17.5 % more real-world success. Tool use, not transport, but it supports scoring a
grasp by the wrench along the carry, not by the lift.

**T4 (Cutkosky; Cutkosky & Howe).** Power grasps (enveloping, palm contact) against precision
grasps; humans choose a power grasp where security matters. A taxonomy, not a parallel-jaw rule.
The only usable content: make the palm a contact.

**T5 (Popov et al.).**

- Oscillation reduces the effective static friction. Tangential oscillation reduces it more than
  normal oscillation of the same amplitude when μ < 1, and oscillation across the sliding
  direction reduces it too. The main mechanism is contact compliance.
- This is real tribology, not simulation. It is the nearest analogue to our creep: the object
  drifts at 200 N per pad, far inside the quasi-static μN limit, while the base jolts the hand.

### What the literature does not give

- **Nothing about a jaw gripper holding an object on a jolting mobile base.** For choosing a
  transport grasp with a parallel jaw, I found nothing beyond what was already covered: Q_DCC,
  Mavrakis effort, Dex-Net's gravity wrench, regrasp toward the CoM.
- **Nothing on grasp depth.** No paper varies how deep the object sits in the jaw and measures
  transport loss. Only T4's preference for palm contact applies.
- **Nothing that models our creep.** In Bullet the creep comes from impulse stepping
  (position-controlled base, few manifold points, no torsional friction; see
  `lift_torque_literature.md` section 6), not from tribology. T5 only shows that real friction
  also weakens under vibration.
- **So calibrate every friction-based limit empirically.** This matches 7.24: the solver's own
  friction limit does not predict a drop.

### Rules to apply or test in a day

**R1 [D from T1]: when the jolt lies along the closing axis, it causes no torque about that axis.**

- Let n be the closing axis, r the CoM offset from it and a the hand's acceleration. The torque the
  pads must hold about n is τₙ = m ((r × (a − g)) · n).
- τₙ is zero when the CoM is on the axis (r ∥ n; that is Q_DCC, which the frozen grasp fixes). It is
  also zero when the part of (a − g) perpendicular to n lies in the plane of r and n.
  - For gravity: the CoM hangs straight below the axis.
  - For fore-aft jolts: n lies along the travel, or r does.
- **Test, with existing poses:** "down, pads along" against "pads across", and "pads forward"
  against "pads sideways". Plate, bleach and strawberry, on the same seeded draws, with the old pose
  as the baseline. Log the pivot angle in the hand per metre.

**R2 [D from T1, Sec. V]: bank the hand so the gravito-inertial vector points into a finger or the
palm, not along a pad.**

- With one finger under, the weight rests on that finger as a normal force. The bleach then creeps
  sideways along it after a bend (7.5).
- In a bend, roll the hand toward the inside by φ = atan(a_lat / g), with a_lat = v²κ taken from the
  route's curvature and the planned speed. This is the waiter's tilt.
- The fore-aft analogue: pitch the hand by atan(a_fwd / g) while speeding up or braking.
- **Test:** bleach and sugar box, "level, finger under", with and without the bank. Measure the
  sideways creep after the first bend (now 43 mm).
- **Caveat [D]:** the per-tick jolts of a position-controlled base average to nothing. Banking can
  cancel only the steady part.

**R3 [D from T2 and T5]: limit and smooth the hand's acceleration, not only its speed.**

- In T2 the acceleration terms set the critical speed, and 120 % of it already slips.
- Our creep rate scales with speed: sugar box 5 mm/m at 15 mm/tick against 2.4 at 11; bleach 3.5
  against 1.1 (7.2, item 3).
- **Test:** cap the change of the base's per-tick step (an acceleration or jerk limit) on the same
  draws. Calibrate the cap per object from the measured creep, not from μ.

**R4 [D from T4 and 7.5]: put a rigid stop downstream of the steady bias.**

- Hand level, the bleach crept into the palm and stopped there (15 mm to 2.6 mm) until gravity took
  it down across the pads. Hand down sends the creep toward the fingertips, the way out.
- Rule: choose the pose whose steady tangential load (gravity plus mean acceleration) points at the
  palm or at a finger, never at the open side of the jaw.
- "Up 30, finger under" is the closest pose we have. Survey 3 dropped it as out of reach.

**R5 (grasp rule, only if the limits are loosened): the plate should hang.**

- Rim-held in a level hand, the plate's CoM sits about one radius out to the side. That is the worst
  gravity lever (sin θ = 1; `lift_torque_literature.md` §7.1), and the plate slid 93 mm in 2 m.
- Folded with the hand kept, it rode 20 m, presumably hanging below the pads [D, not checked in the
  logs].
- Rule: choose a rim point and pose that put the CoM straight below the pads. Then, by R1, put n
  along the travel, so only the bends load the weak axis, and slow the bends.

**R6 (grasp rule): bleach and the 10 cm can.** Q_DCC first: it is the only published rule with
evidence (Kolamuri: more grip force "does little"). Then seat the object against the palm (T4).
There is no published number for the depth.

---

## 2. Caging and enveloping holds for small or round objects

### Sources

| # | Source | Checked |
|---|---|---|
| C1 | J. Mahler, F. T. Pokorny, Z. McCarthy, A. F. van der Stappen, K. Goldberg, "Energy-Bounded Caging: Formal Definition and 2-D Energy Lower Bound Algorithm Based on Weighted Alpha Shapes", IEEE RA-L 1(1), 2016, https://goldberg.berkeley.edu/pubs/Energy-Bounded-Caging-RA-Letters-2016.pdf; J. Mahler, F. T. Pokorny, S. Niyaz, K. Goldberg, "Synthesis of Energy-Bounded Planar Caging Grasps Using Persistent Homology", IEEE T-ASE 15(3):908–918, 2018, doi:10.1109/TASE.2018.2831724 | [abs]; author list of the 2016 paper [mem] |
| C2 | A. Rodriguez, M. T. Mason, S. Ferry, "From Caging to Grasping", IJRR 31(7):886–900, 2012, doi:10.1177/0278364912442972 (RSS 2011) | [abs] |
| C3 | E. D. Rimon, F. T. Pokorny, W. Wan, "Geometric characterization of two-finger basket grasps of 2-D objects: contact space formulation" (KTH copy, link now 404); A. Shirizly, E. D. Rimon, W. Wan, "Contact Space Computation of Two-Finger Gravity Based Caging Grasps Security Measure", IEEE RA-L 2021 | [abs]; venue and year of the first [not verified] |
| C4 | F. T. Pokorny, J. A. Stork, D. Kragic, "Grasping objects with holes: A topological approach", ICRA 2013, pp. 1100–1107 | [abs] |
| C5 | Y. Dong, X. Cheng, F. T. Pokorny, "Characterizing Manipulation Robustness through Energy Margin and Caging Analysis", IEEE RA-L 2024, https://arxiv.org/abs/2404.12115 | [abs] |

### What each one actually claims

**C1 (energy-bounded caging).**

- In a field such as gravity, a cage need not be complete. An energy-bounded cage is one the object
  can leave only by gaining a given energy (a rise of its CoM).
- EBCA-2D gives a lower bound on that escape energy. EBCS-2D searches for the most robust cages.
  In 2018 it was tested on 7 planar parts and 4 gripper types, taking 2 to 3 min on 6 cores with
  200,000 samples.
- The physical runs were planar pushing with parallel jaws, not carrying. Planar only.

**C2 (Rodriguez, Mason & Ferry).** With two fingers, every cage is a "pregrasping cage": the
fingers can close to a grasp without breaking it (not so with more fingers). Our use is the
converse: a pinch that loses friction falls back to a cage only if the cage existed around it.

**C3 (two-finger basket grasps).** A basket grasp holds an object against gravity on two finger
bodies, in equilibrium, with no squeezing. Its security is the basket's depth: the rise of the CoM
to the "puncture" point, where escape costs the least gravitational energy. 2-D only.

**C4 (objects with holes).** Objects with a hole (handles, cups) can be grasped topologically, by
closing a loop through the hole. The mug's handle is such a hole.

**C5 (energy margin).** Extends caging and an "energy margin to failure" to dynamic manipulation,
using a kinodynamic planner, and validates in simulation and on hardware. From the abstract alone I
cannot tell whether it covers transport disturbances.

### What the literature does not give

- **Nothing three-dimensional and practical for a two-pad jaw.** The caging synthesis results are
  planar (C1, C3).
- **[D] A pinch of a round object is friction-only.** Two parallel flat pads cannot cage a sphere:
  the strawberry can roll or slide out within the pads' plane with no rise of its CoM, so the
  escape energy is zero in any orientation. A cage needs a third surface, the palm or the lower
  finger acting as a floor.
- **No caging result for a moving hand.** Re-orienting the hand is where we lose the strawberry and
  the mug (7.24: of the first carries lost, 11 of 69 and 20 of 57 went while the pose was being
  taken). It sweeps gravity through the hand's frame, and no source checks the cage along that
  path.
- **No measurement under per-tick impulses** like Bullet's.

### Rules to apply or test in a day

**K1 [D from C1 and C3]: compute the basket depth along the whole re-orientation, not only at the
end pose.**

- Cheap 1-D proxy: in the hand's frame, the height of the CoM below the rim of the support, along
  gravity.
  - Hand up, object on the palm: about the finger length.
  - Hand down: zero.
  - Finger under: the lip of the lower finger.
- **Test:** for the strawberry and mug, log the proxy every tick while the pose is taken, and check
  whether the losses fall where it reaches zero.
- If they do, order the way in so the proxy stays positive: rotate about the axis that keeps the
  lower finger or the palm under the object.

**K2 [D from C3 and T4]: carry small round objects with the support under them and the hand tilted
so gravity points into it.** "Up 30, finger under" is a basket; "hand down" is the opposite. Survey
3 lost the strawberry while taking the finger-under pose, so by K1 fix the way in, not the pose.

**K3 [D from C4]: the mug, by its handle** (a grasp change, so it waits for the freeze to lift). A
pad closed around the handle loop makes a topological cage. Until then, by R1, keep the mug's CoM
straight below the closing axis through the re-orientation.

**K4 [D]: the marker.** Pinched across its axis, a thin cylinder escapes along that axis with no
rise of the CoM when the axis is horizontal. "Hand down, pads along" carried it the whole route in
both surveys: the marker hangs along gravity, so neither gravity nor fore-aft jolts load the closing
axis (R1). Keep that pose. Its 7.23 losses are mostly the frozen pick (lifted by tick 600 in 17 of
100).

**K5: holds that do not rely on friction.** Beyond basket and topological cages, the literature has
nothing for a flat two-pad gripper. A three-contact hold, one pad pressing the object against the
palm, is the only one within reach. It still relies on the closing force, though not on friction
across the pads.

---

## 3. Order of tests (each on the same seeded draws, old against new)

1. **R1 pad-line check** with existing poses: `pads="forward"` against `"sideways"`, and
   `steer="along"` against `"across"`. Plate, bleach, strawberry, mug.
2. **K1 depth proxy, logged only.** No change of behaviour. Strawberry and mug, the ways into the
   poses.
3. **R2 bank in bends.** Bleach and sugar box, finger under.
4. **R3 acceleration or jerk cap on the base step.** All 15, because it changes every carry.

R5, R6 and K3 change the grasp: leave them until the grasp limits are loosened.
