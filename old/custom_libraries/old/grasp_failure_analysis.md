# Fetch grasp planner: failures, limitations and repair priorities

Reviewed: 17 November 2026

## Scope and evidence

This report consolidates the problems identified in `grasp(1).py`, including its relationship to the supplied Fetch model, MoveIt and D(R,O) Grasp. It describes failure mechanisms; it does **not** claim that every mechanism occurred in the user's run.

Files reviewed:

- `grasp(1).py`: 871 lines; line numbers below refer to this uploaded version.
- `fetch/fetch.urdf` inside `fetch.zip`.
- `fetch/fetch.srdf` inside `fetch.zip`.

Evidence labels:

- **Confirmed implementation property:** directly established by reading the code or robot description. Its effect in the user's scene may still be conditional.
- **Reproduced limitation:** demonstrated with an isolated numerical check, without the full robot environment.
- **Conditional failure:** a plausible consequence requiring the actual starting state, scene, controller or logs to establish.
- **Intentional restriction:** a legitimate design choice that reduces the set of plans the implementation can find.

The sampler and follower were exercised in isolation using NumPy/SciPy. The SRDF exclusions were compared programmatically against the planner's pair-selection rules. Full PyBullet execution was **not** performed: PyBullet was unavailable, and an installation attempt could not obtain a package. The environment/controller implementation, actual observations and failed-run logs were not supplied. No original source files were changed.

## Main findings

The most important problems to investigate are:

1. The planner does not automatically load the robot's supplied self-collision exclusions.
2. A grasp can be discarded after the first pose-valid IK solution collides, without searching alternative IK branches for that grasp.
3. Candidate ranking can exclude every side grasp from the attempted set.
4. The torso is frozen, even when changing its height could make the grasp reachable.
5. Planned finger closure permits contact but does not establish the intended bilateral contact geometry.
6. Lift planning assumes rigid attachment without establishing physical holding capability.
7. Execution can depart from the checked path and lacks explicit trajectory timing.

**The use of a midpoint is not, by itself, a demonstrated bug.** For this symmetric parallel gripper, midpoint, closing direction and gap determine the two nominal pad contact locations. Explicit finger-to-point optimisation would improve the formulation and its validation, but it will not repair collision configuration or motion-search failures.

## 1. Robot configuration and collision checking

### F01 — Supplied SRDF exclusions are not loaded automatically

**Evidence:** Confirmed configuration mismatch under default construction; runtime effect conditional.

**Code:** `GraspPlanner.__init__()`, lines 282–359; `_valid()`, lines 544–551.

The constructor defaults to `disabled_self_pairs=()`. It builds exclusions for fixed clusters and direct joint neighbours, but does not read `fetch.srdf`. Unless the caller supplies the SRDF pairs explicitly, five pairs excluded by the supplied SRDF remain checked:

| Link 1 | Link 2 |
|---|---|
| `shoulder_lift_link` | `torso_lift_link` |
| `wrist_roll_link` | `forearm_roll_link` |
| `l_gripper_finger_link` | `r_gripper_finger_link` |
| `upperarm_roll_link` | `head_pan_link` |
| `upperarm_roll_link` | `head_tilt_link` |

The failure occurs when one of these pairs is within the configured collision margin and `_valid()` rejects it. If this happens at the measured starting state, changing the grasp candidate cannot resolve it.

The static comparison proves that the pair policies differ. It does not prove these pairs intersect in the user's starting pose, nor that the calling code has not already supplied the exclusions.

**Repair:** Load and validate the intended SRDF exclusions, pass them into the planner, and print rejected link **names**, distances and stages. Review exclusions against the modified model rather than disabling all self-collision checks. MoveIt explicitly supports SRDF collision semantics [S2].

### F02 — Every candidate can fail during opening because the starting arm is invalid

**Evidence:** Confirmed control flow; actual collision conditional.

**Code:** `plan()`, lines 687–692; `edge_samples()`, lines 93–97.

The opening check samples the whole motion from the measured state to the opened-finger state, including the measured state itself. It checks all relevant arm collisions, not just the fingers.

An elbow too close to an obstacle, a self-collision policy mismatch, or another invalid starting condition can therefore produce repeated `opening_fail` results even though finger opening is not the cause.

**Repair:** Validate the start once and report its exact failure before evaluating candidates. A genuinely invalid start needs a separately handled recovery or repositioning motion; it should not be hidden behind repeated grasp failures.

### F03 — Collision margins and geometric tolerances leave little robustness

**Evidence:** Confirmed numerical settings; practical effect conditional.

**Code:** `GraspConfig`, lines 46–74; `_valid()`, lines 505–570.

| Setting | Default |
|---|---:|
| Extra open gap | 8 mm total, or 4 mm per side |
| Collision margin | 2 mm |
| IK position tolerance | 3 mm |
| IK orientation tolerance | 0.04 rad, approximately 2.3 degrees |
| Permitted finger penetration during contact phases | 1.5 mm |
| Squeeze beyond nominal contact | 0.5 mm per finger |

In a simple centred parallel-face case, the 4 mm opening clearance per side leaves only about 2 mm beyond the required margin. A permitted IK position error can consume this allowance. Collision checking may correctly reject the resulting pose, but the overall design can be fragile. Orientation and tracking errors add further displacement.

**Repair:** Choose tolerances together based on measured geometry and tracking error. Improve contact accuracy and clearance before simply increasing permitted penetration or removing collision margins.

### F04 — Collision checking is discrete

**Evidence:** Confirmed design limitation, explicitly acknowledged in the file header.

**Code:** `edge_samples()`; `RRTConnect.edge_valid()`; `_cartesian()`.

The planner checks sampled joint configurations, using a maximum per-joint increment of 0.025 rad for arm edges. It does not continuously check every intermediate robot pose. Joint-angle resolution also does not directly bound the displacement of every point on the robot.

Thin obstacles or brief collisions between samples can be missed. Conversely, reducing the step size increases query cost and consumes the search budget.

**Repair:** Select resolution using workspace clearance and link motion, or use a suitable continuous/adaptive collision-checking method. Keep execution tracking error within the clearance assumptions.

## 2. Grasp sampling, IK and search

### F05 — Candidate ranking can starve side grasps

**Evidence:** Reproduced limitation.

**Code:** `sample_antipodal_grasps()`, lines 419–449; `plan()`, line 672.

The score rewards downward approaches:

```python
score = quality[k] + 0.10 * (-approach[2])
```

The sampler returns up to 500 candidates by default, but `plan()` considers only the first 60. A synthetic 4 cm cube with 600 uniformly sampled face points and exact outward normals produced:

| Set | Count | Downward: approach z < -0.95 | Sideways: absolute approach z < 0.1 |
|---|---:|---:|---:|
| Returned candidates | 500 | 80 | 154 |
| First 60 eligible for planning | 60 | 60 | 0 |

Side grasps existed but were never attempted. This result is specific to that synthetic input, not a claim about the user's actual object.

**Repair:** Allocate candidate slots across approach directions, contact locations and orientations before sorting within those groups. Include reachability in prioritisation. Raising the candidate count alone can still waste most of the budget on similar grasps.

### F06 — The first geometrically valid IK solution can terminate the search for a grasp too early

**Evidence:** Confirmed control-flow limitation.

**Code:** `_ArmIK.solve()`, lines 246–278; `plan()`, lines 697–720; `RRTConnect.plan()`, lines 124–135.

IK checks limits and end-effector pose error, but not scene collisions. It returns the first pose-valid solution. RRT then checks that solution as its goal. If the goal collides, RRT returns `None`, and `plan()` advances to another grasp candidate.

The seven-joint arm may have another solution for the same hand pose with a different elbow configuration. The current loop does not systematically search those alternatives after a collision rejection. The configured IK restart count mainly helps find a pose solution; it does not ensure multiple collision-valid branches are explored.

**Repair:** Search multiple distinct IK solutions per candidate, validate each, and plan to feasible alternatives. Separate `goal_in_collision` from a path-search timeout. More RRT time cannot make an invalid endpoint valid.

### F07 — Freezing the torso reduces reachability

**Evidence:** Intentional restriction with conditional failures.

**Code:** `_ArmIK.__init__()`, lines 179–239; `_full()`, lines 572–575.

The private model freezes every non-arm joint at the measured configuration. Only seven arm joints participate in IK. Torso height is preserved through planning and execution.

A reachable Fetch grasp may be unreachable for this restricted arm-only problem. The pregrasp may also be unreachable even when the final grasp is reachable.

**Repair:** Where task rules allow it, consider torso height during grasp planning or reposition before arm planning. Fetch's documented MoveIt examples use an `arm_with_torso` group [S1]. Do not enable torso motion without updating collision checking and execution consistently.

### F08 — Approach orientations, pregrasp distance and lift direction are restricted

**Evidence:** Intentional search restrictions.

**Code:** `sample_antipodal_grasps()`, lines 423–433; `plan()`, lines 695–696 and 736–746.

For each pair, the sampler tries five rotations about the closing axis and two finger-axis flips. Planning then requires an 8 cm straight approach and, by default, an 8 cm vertical lift with fixed hand orientation.

A grasp can be rejected because it needs another hand orientation, a shorter approach, or a retreat before lifting. A successful approach and closure are still discarded if the prescribed lift fails.

**Repair:** Expand orientations selectively, adapt pregrasp distance, and consider alternative retreat/lift strategies. Setting `lift_distance=0` can isolate planning stages during diagnosis, but does not demonstrate that the object can be lifted.

### F09 — Cartesian continuation can fail at singularities or branch changes

**Evidence:** Confirmed restrictions; scene-dependent failure.

**Code:** `_cartesian()`, lines 587–626.

Each 5 mm Cartesian step uses only one IK restart, seeded from the previous solution. Any solution with a maximum joint jump above 0.18 rad is rejected.

Near a singularity, a small hand displacement may require a large joint change. Another branch may exist, but the local continuation may not find it. Joint interpolation between solved hand poses also does not make the entire hand path exactly straight.

**Repair:** Adapt Cartesian step length, consider alternative branches or approach orientations, and distinguish IK failure from a jump-limit rejection. Large jumps should be handled through checked planning, not accepted blindly.

### F10 — Search budget exhaustion is not proof of infeasibility

**Evidence:** Intentional finite search.

**Code:** `GraspConfig`, lines 57–73; `RRTConnect.plan()`; `plan()`.

Defaults allow 60 attempted candidates, 1 second of RRT per candidate, 2,500 RRT iterations and 30 seconds overall. Continuous-like arm joints are searched in a finite interval around the measured start. Multiple collision queries and failed candidates can consume the budget before useful alternatives are attempted.

The total deadline is also not a hard real-time bound on every operation: sampling and model setup do not continuously check it, and an individual simulator query can finish after a deadline.

**Repair:** Record time per stage and distinguish endpoint rejection, IK failure, local RRT timeout and global timeout. Improve ordering and branch selection before increasing budgets.

### F11 — Point-cloud assumptions can prevent candidate generation

**Evidence:** Confirmed input contract; whether violated is unknown.

**Code:** `sample_antipodal_grasps()`, lines 384–449.

The algorithm requires world-frame points with correctly oriented outward normals. Invalid values are filtered and normals are normalised, but their frame and outward orientation are not verified. A partial cloud may omit the opposite surface needed for an antipodal pair. Sampling at most 800 contact points can omit useful regions, and the discrete orientation/obstruction tests can further remove candidates.

The width filter requires the object to fit with opening clearance. With the uploaded defaults, the maximum accepted contact separation is 94.85 mm, even though the maximum geometric gap is 102.85 mm.

**Repair:** Validate frames, normal signs, cloud coverage and units. Log how many pairs survive each filter. A `no_antipodal_pairs` result can also mean that downstream pose filtering produced no candidates; inspect `sample_stats` to distinguish them.

## 3. Contact geometry and physical grasping

### F12 — Finger contact locations are not explicit IK acceptance constraints

**Evidence:** Confirmed formulation limitation.

**Code:** `plan()`, lines 681–696; `_ArmIK.solve()`; `_live_pose_ok()`.

The code takes the contact midpoint, rotates the gripper so its closing axis follows the point pair, calculates a nominal opening, and solves IK for `gripper_link`. It checks the gripper-link pose rather than both contact-point residuals.

For the ideal model, this geometric construction is consistent with the contact points. However, permitted pose error and model mismatch can move the actual pad locations away from their targets.

The professor's requested finger-to-point formulation can be expressed as:

```text
minimise ||f_right(q) - p_right||² + ||f_left(q) - p_left||²
```

Here `f_right` and `f_left` are forward-kinematic positions of specified points on the inner pad surfaces. They are **not** the finger-link origins. Both fingers must be solved jointly because they share the arm. The intended correspondence must account for the candidate's finger-axis flip.

For the uploaded URDF and the configured contact x-offset, the nominal local surface points are `(0.025, -0.014, 0)` in the right finger link and `(0.025, +0.014, 0)` in the left finger link. These values follow from the box collision offsets and half-thickness; different contact locations or models require recalculation.

**Repair:** Use the midpoint construction as an initial guess, then solve or refine both surface-point targets and require each residual to pass. Retain approach-orientation and collision constraints; two position targets alone leave rotation about their connecting line unresolved.

### F13 — Planned closure allows contact without requiring bilateral contact

**Evidence:** Confirmed validation gap.

**Code:** `_valid()`, lines 531–543; `plan()`, lines 726–735.

With `touch=True`, shallow finger contact is allowed. Absence of contact is not rejected. Closure is checked up to nominal opening, but no terminal condition proves that both intended pad surfaces touch the object.

The extra squeeze is only 0.5 mm per finger. A small offset can leave one side clear or cause asymmetric contact; whether the object then centres, moves away or stays wedged depends on physics.

**Repair:** Validate bilateral contact geometry, locations and normal directions at nominal closure. During execution, use measured contact and suitable gripper force/control settings. Do not treat an absence of forbidden collisions as evidence of a grasp.

### F14 — Antipodal screening does not establish sufficient holding force

**Evidence:** Confirmed physical modelling limitation.

**Code:** `sample_antipodal_grasps()`; `GraspConfig.friction_coefficient`; `GraspFollower._contacts()`.

The sampler tests alignment with assumed friction cones. It does not calculate required contact forces, available motor forces, centre-of-mass torque or a full grasp-wrench feasibility condition. The assumed coefficient of 0.5 is not read from the simulated contact settings.

In a simplified vertical-sided pinch with equal horizontal normal forces `N`, supporting weight requires at least `2 * mu * N >= m * g`. This is an illustrative necessary force balance for that arrangement, not a general grasp-stability test.

**Repair:** Check actual controller force limits, friction and object mass. Include torque resistance and contact placement where relevant, and verify the grasp dynamically.

### F15 — Squeeze targets do not define or guarantee gripping force

**Evidence:** Confirmed control dependency.

**Code:** `plan()`, line 685; `GraspFollower.step()`, lines 835–842.

The target is slightly inside the nominal contact configuration. Actual force depends on the environment's position-controller gains, force limits, timestep and contact response. This file does not configure those values. The URDF's finger effort declaration alone does not establish the motor command used by the environment.

**Repair:** Inspect `env.step()` and the motor-control setup. Tune and validate contact force and closing speed using the actual simulated controller rather than treating a position offset as a specified force.

### F16 — Lift planning assumes the object remains rigidly attached

**Evidence:** Confirmed planning assumption; grip failure conditional.

**Code:** `_valid()`, lines 524–529 and 552–569; `plan()`, lines 737–743.

The planner computes a fixed gripper-to-object transform, then temporarily moves the object with the hand during collision checks. This checks a hypothetical carried object's clearance. It does not simulate whether the fingers can hold it.

An accepted lift may slip or drop during execution. Conversely, the prescribed rigid transform can reject a lift that would require a different object settling pose or retreat strategy. Shallow pre-existing support contacts are handled specially, but that is not a dynamic model of lifting off a support.

**Repair:** Keep carried-object collision checking, but validate the real grip before committing to the lift and monitor relative object motion. Planning-scene attachment and physical attachment are different concepts.

### F17 — Bilateral contact detection is a weak success criterion

**Evidence:** Confirmed check; physical effect conditional.

**Code:** `_contacts()`, lines 785–790; `step()`, lines 846–864.

The test requires each finger link to have some contact with normal force above 0.01. It does not require contact at the selected points, on the intended pad faces, with opposing normals, or with enough force to carry the object.

Twelve consecutive contact checks and a minimum of thirty close-phase ticks reduce transient acceptance, but do not prove a stable grasp. The final lift check requires only half the configured lift height, so a nominal 8 cm lift needs at least 4 cm of object rise to pass that condition.

**Repair:** Monitor contact locations, force levels, object-to-hand motion and the required lift outcome. Define task success separately from contact presence.

## 4. Execution and state monitoring

### F18 — The follower can leave the collision-checked path

**Evidence:** Reproduced limitation.

**Code:** `GraspFollower.step()`, lines 833–843 and 864–868.

The common interpolation factor puts the next command on the line from the **measured** state to the current target. It does not ensure that this line is a planned edge. The follower advances to the next waypoint when each arm joint is within 0.008 rad, before the previous endpoint is necessarily reached.

In an isolated two-arm-joint example, the next checked edge had joint 1 fixed at zero and joint 2 moving from zero to 0.025. With residual joint-1 tracking error of 0.006, the follower commanded `[0.0024, 0.015]`, outside that edge.

This demonstrates an incorrect guarantee in the interpolation comment, not a demonstrated collision in the full scene. Even an on-path command does not prove the physical controller follows that path exactly.

**Repair:** Track deviation from the planned trajectory, use tolerances compatible with clearance, and stop or replan when those bounds are exceeded. Avoid treating a near waypoint as an exact corner without checking the resulting transition.

### F19 — No explicit time parameterisation or acceleration limits

**Evidence:** Confirmed omission in this file.

**Code:** `GraspFollower.step()`, lines 835–842.

Increment limits are specified per call, without a timestep. The generic limit is 0.015 for non-finger coordinates; finger increments are 0.0004 during closing and 0.002 otherwise. Pinned joints are then assigned their target directly.

These are command increments, not validated velocities or accelerations. Behaviour changes with calling frequency and motor response. Physical limits may exist in the environment, but this file does not use them to produce a timed trajectory.

**Repair:** Parameterise paths against joint velocity and acceleration limits, execute using known timing, and validate tracking. MoveIt provides this trajectory-processing capability [S3].

### F20 — Parked-state and object-motion tolerances are not tied to clearance

**Evidence:** Confirmed numerical checks; failure conditional.

**Code:** `GraspFollower.step()`, lines 826–832.

The parked-joint check uses a single numeric threshold of 0.03 across prismatic and angular coordinates. This can allow centimetres of translation before failing, despite a 2 mm planning margin. It also mixes metres and radians in one maximum comparison.

Before closure, the object can move up to 12 mm or rotate up to 0.10 rad without triggering the movement guard. These tolerances can exceed the narrow contact clearances and squeeze distance. Passing the guard does not mean the original grasp remains valid.

**Repair:** Use joint-specific tolerances and world-space hand/object errors linked to the collision and contact budget. Revalidate or replan when the scene changes materially.

### F21 — Timeout and contact thresholds depend on tick frequency

**Evidence:** Confirmed implementation property.

**Code:** `GraspFollower.step()`, lines 846–854.

The follower uses 12 stable-contact ticks, at least 30 closing ticks, more than 20 lost-contact ticks, and more than 600 ticks for a per-waypoint timeout. Their physical durations depend on how many simulation steps occur per call. The timeout resets at each waypoint, so it is not a bound on total execution time.

**Repair:** Express monitoring windows in simulation time or seconds and retain an overall execution timeout. Log both elapsed time and tick counts.

### F22 — Small post-validation clipping changes the commanded configuration

**Evidence:** Confirmed bounded consistency issue, not a demonstrated collision.

**Code:** `_configure_measured_limits()`, lines 466–498; `plan()`, line 759; `step()`, line 868.

The planner permits small measured torso/finger limit overshoots and collision-checks that measured geometry. Returned targets are clipped to the original limits. Thus a checked parked coordinate can differ slightly from the commanded coordinate. The allowance is deliberately small, but it still changes the configuration after validation.

**Repair:** Check the transition to any corrected in-limit parked state, or make that correction a separately validated initial step. Do not solve this by clipping arbitrary IK solutions, which the current IK code correctly avoids.

### F23 — Failure handling does not provide retraction or recovery

**Evidence:** Intentional scope limitation.

**Code:** `GraspFollower._fail()`, lines 792–804.

On failure, the follower holds the measured state and retains a closing target if it was closing or lifting. This is not a planned safe retraction, object placement or navigation-ready pose. The file explicitly warns against automatically navigating away with an extended arm.

**Repair:** Add recovery as a separate checked operation, conditioned on whether an object is held and which motions remain feasible. A caller must not interpret `failed` as permission to navigate immediately.

## 5. Input guards and diagnostic limitations

### F24 — Validity guards can stop planning before the grasp search

**Evidence:** Confirmed guards, not necessarily bugs.

**Code:** `_validate_fingers()`; `_configure_measured_limits()`; `plan()`, lines 635–649; `_live_pose_ok()`.

The code can reject or raise on stale observations, malformed `qpos`, excessive measured joint-limit violations, unsupported non-arm joints, missing URDF assets, incompatible finger geometry or disagreement between private and live kinematics.

Arm measured limits receive effectively numerical tolerance only, while torso and finger overshoot tolerances are explicit. A small measured arm limit violation can therefore terminate a run that appears visually reasonable.

**Repair:** Report the exact guard failure and correct its source. Do not bypass limits or model-mismatch checks merely to get a plan. There is no evidence in the supplied files that every guard is currently failing.

### F25 — Diagnostics do not fully separate root causes

**Evidence:** Confirmed reporting limitation.

**Code:** `_record_invalid()`, lines 500–503; `plan()`, lines 644–758; debug prints in `_cartesian()` and `step()`.

Only a few invalid examples are retained, and `_record_invalid()` is used for opening and closure. RRT can return `None` for an invalid endpoint or search timeout. A printed last rejected state can be an incidental search sample rather than a proof that no path exists. Cartesian IK exhaustion can reflect either solver failure or elapsed budget. Candidate count alone does not reveal orientation coverage.

**Repair:** Record per-candidate orientation, target pose, IK branch, endpoint validity, rejection stage, named links, distances and time consumed. Preserve structured results for all stages instead of relying only on console prints.

## 6. What the uploaded URDF confirms

The finger geometry matches the default model assumptions:

| Property | Uploaded model | Planner default |
|---|---:|---:|
| Collision pad length | 0.063 m | 0.063 m |
| Collision pad height | 0.024 m | 0.024 m |
| Maximum travel per finger | 0.05 m | 0.05 m |
| Calculated closed gap | 0.00285 m | 0.00285 m |
| Closing-axis directions | Opposing local Y axes | Symmetric Y-axis model |

Consequently, there is no basis from these files alone to blame the default pad dimensions or closed-gap formula. The configured contact offset of 0.025 m lies within the pad half-length of 0.0315 m. The remaining depth ahead of that nominal contact location is only 6.5 mm, which is relevant when choosing contact and tracking tolerances, but is not by itself an error.

## 7. MoveIt and D(R,O) Grasp comparison

| Component | Current custom implementation | Role of an alternative |
|---|---|---|
| Contact selection | Antipodal point-pair sampling | Retain or replace with another grasp generator |
| Contact-to-configuration conversion | Gripper-link IK with derived gap | Explicit surface-point constraints or a richer hand model |
| Robot semantics | Custom adjacency and exclusions | MoveIt loads configured URDF/SRDF semantics [S2] |
| Search | One accepted pregrasp IK result followed by custom RRT | Use a configured motion pipeline and suitable IK search |
| Torso | Frozen | Fetch's documented example uses `arm_with_torso` [S1] |
| Timing | Untimed per-call position increments | MoveIt supports velocity/acceleration-aware time parameterisation [S3] |
| Grip physics | Depends on simulator/controller | Must still be established through execution and feedback |

D(R,O) Grasp predicts hand–object distances, reconstructs desired hand points, estimates link poses and recovers joint configurations. The stated final joint optimisation aligns link translations through Jacobian updates; it is not simply a two-fingertip contact solver. Its real-world deployment separately uses MPLib for arm motion and a PD controller [S4]. These methods address related but distinct parts of the task.

Using MoveIt does not automatically choose stable contacts or establish frictional attachment. It also needs the actual scene and a working controller interface. The supplied ZIP is a robot model, not a complete working MoveIt deployment: its SRDF contains collision exclusions but no planning groups. The modified base joints and finger shapes must agree across the simulator and planning model. Fetch's cited official tutorial is for ROS Noetic; compatibility and the PyBullet bridge must be resolved before migration [S1].

## 8. Failure-log interpretation

| Result or log | What has failed | First investigation |
|---|---|---|
| `start_joint_limit_violation` | Measured state acceptance | Joint name, excess and allowed tolerance |
| Stale observation exception | Observation/live-state consistency | Acquire after navigation and simulation stop |
| `no_antipodal_pairs` | Candidate generation returned nothing | Pair statistics, normals, cloud coverage, width and pose filters |
| `[FAIL opening]` | Starting state or finger-opening edge | Named collision pair; SRDF policy |
| `[FAIL pregrasp IK]` | Pregrasp pose solving | Reachability, torso height, orientation, time remaining |
| `[FAIL transit]` | Endpoint validity or route search | Test start and goal separately; try another IK branch |
| `[FAIL approach] Cartesian IK` | Local Cartesian continuation | Reachability, singularity, seed and deadline |
| `[FAIL approach] Joint jump` | Step-continuity threshold | Alternate branch, smaller step, alternate approach |
| `[FAIL approach] Invalid path state` | Approach validity | Named links, object/table clearance and distance |
| `[FAIL closure]` | Geometric closing edge | Finger placement, palm collision and penetration |
| `[FAIL lift]` | Hypothetical carried-object lift | Payload/support collision, IK and lift direction |
| `Timed out during close` | Sustained bilateral contact not established | Surface-point residuals and motor/contact settings |
| `Lost bilateral finger contact during lift` | Grip contact lost | Slip, available force, torque and tracking |
| `Arm reached lift pose but the object did not rise` | Object transport failed | Physical grip, not just arm IK |
| Parked-state movement failure | Frozen-state assumption violated | Base/torso/head motion and controller response |

## 9. Recommended repair order and verification

1. **Establish a valid start.** Load the intended collision exclusions, resolve link names, and run one explicit measured-state validity check. Verify both an intended exclusion and a genuine forbidden collision.
2. **Make IK collision-aware at the candidate-search level.** Retain multiple branches and test whether a candidate can succeed after its first branch is rejected.
3. **Diversify the attempted candidates.** Repeat the synthetic cube check and verify that the attempted subset includes top-down and side approaches.
4. **Implement and validate explicit finger-to-point targets.** Evaluate FK at the inner surface points, check both residuals, and retain orientation and collision requirements. Test known reachable contact pairs before noisy clouds.
5. **Allow appropriate reachability adjustments.** If permitted, include torso height or plan repositioning. Test this separately from contact generation.
6. **Validate real closure.** Measure contact positions and forces in the actual environment, confirm suitable controller settings and test lifting with the intended object.
7. **Use timed, monitored execution.** Bound velocities, accelerations and tracking error. Repeat the off-path follower example and confirm the corrected execution policy handles it.
8. **Add structured diagnostics and recovery.** Preserve stage-specific causes and make retraction/navigation readiness explicit.

Success at one stage should not be presented as proof of success at later stages. A good contact pair can have no reachable arm pose; a reachable pose can have no collision-free route; a valid route can execute poorly; bilateral contact can still fail to hold the object.

## 10. Reproduction details for isolated checks

### Candidate-ranking check

The following runs the supplied geometric sampler without PyBullet. Use the reviewed source file as `source`.

```python
import importlib.util
import sys
import numpy as np
from scipy.spatial.transform import Rotation

source = "grasp(1).py"
spec = importlib.util.spec_from_file_location("grasp_review", source)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)

rng = np.random.default_rng(42)
points, normals = [], []
for axis in range(3):
    for sign in (-1, 1):
        face = rng.uniform(-0.02, 0.02, (100, 3))
        face[:, axis] = sign * 0.02
        normal = np.zeros_like(face)
        normal[:, axis] = sign
        points.extend(face)
        normals.extend(normal)

planner = module.GraspPlanner()
candidates = planner.sample_antipodal_grasps(
    points=np.asarray(points), normals=np.asarray(normals)
)
approach = np.asarray([
    Rotation.from_quat(candidate[3]).as_matrix()[:, 0]
    for candidate in candidates
])
for label, subset in (("returned", approach), ("attempted", approach[:60])):
    print(label, len(subset),
          "downward", np.count_nonzero(subset[:, 2] < -0.95),
          "sideways", np.count_nonzero(np.abs(subset[:, 2]) < 0.1))
```

Observed output: `returned 500 downward 80 sideways 154`; `attempted 60 downward 60 sideways 0`. Tie ordering and numerical-library differences can affect exact counts; the check tests orientation coverage, not a full grasp success rate.

### Follower check

Two planned arm waypoints were `[0, 0]` and `[0, 0.025]`, with two dummy finger coordinates and one pinned coordinate. The first observation had arm coordinates `[0.007, 0]`, which passed the 0.008 waypoint tolerance. The next observation assumed residual tracking lag `[0.006, 0]`.

For the next target, `error = [-0.006, 0.025]` and `scale = 0.025 / 0.015`. The issued arm command is `[0.0024, 0.015]`. Joint 1 is nonzero although the checked next edge has joint 1 identically zero. This observation sequence was supplied to the real follower implementation with minimal mock planner/plan data; it was not a simulated robot rollout.

### File fingerprints

```text
grasp(1).py
SHA-256: 9316dc82937854935466b29e95e338c1b849d60208f0fd24edb9278109a71434

fetch/fetch.urdf
SHA-256: 742a88607b0c110bb281b27f11ad0d7b7ab8f9adfbc30b69d31fe18aa1ea7ff1

fetch/fetch.srdf
SHA-256: 179e1cdc27f763d2ff9d81ef3d1185692954b68171c0ad785a9d991099bfce87
```

## References

- **[S1]** [Fetch manipulation tutorial — ROS Noetic](https://fetchrobotics.github.io/docs/manipulation.html). Official Fetch MoveIt launch/configuration examples and the arm-with-torso planning group.
- **[S2]** [MoveIt URDF and SRDF documentation](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html). Robot semantics, groups and self-collision exclusions.
- **[S3]** [MoveIt trajectory processing](https://moveit.picknik.ai/main/doc/concepts/trajectory_processing.html). Time parameterisation with joint velocity and acceleration limits.
- **[S4]** [D(R,O) Grasp, arXiv full text](https://arxiv.org/html/2410.01702v2), especially Section III-C and Appendix A.2. The linked project PDF exceeded the web reader's size limit; the method and deployment discussion were checked in this arXiv version.
- **[S5]** [MoveIt Task Constructor pick-and-place tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html). Explicit approach, closure, contact permissions and planning-scene attachment stages.

External references describe the comparison systems. Findings about the uploaded implementation come from its source and the isolated checks described above.
