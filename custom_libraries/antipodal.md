The file combines grasp generation, inverse kinematics, motion planning, and feedback control. Each solves a different part of the problem:
Component	Question it answers
Antipodal sampling	Where should the fingers contact the object?
Gripper pose construction	Where should the gripper be, and which way should it point?
Inverse kinematics	Which arm joint angles achieve that pose?
Collision checking	Is this configuration or movement obstructed?
Motion planning	How should the arm travel to the grasp?
Execution and verification	Did the robot actually grasp and lift the object?


1. Antipodal contact-pair sampling — sample_antipodal_grasps()

The input is your observation’s object point cloud and surface normals. The algorithm examines pairs of points as possible locations for the two opposing fingers.

For contact points \(\mathbf p_1,\mathbf p_2\), it calculates:
\[
w=\|\mathbf p_2-\mathbf p_1\|,
\qquad
\mathbf a=\frac{\mathbf p_2-\mathbf p_1}{w}
\]
Here, \(w\) is the required opening and \(\mathbf a\) is the closing axis, pointing from one contact toward the other.

A pair survives only if:
- Its width fits within the configured gripper range.
- Its normals face approximately opposite directions.
- The line joining the contacts lies inside both friction cones.
- The closing axis is approximately horizontal, avoiding top–bottom pinches through the table.
For outward surface normals \(\mathbf n_1,\mathbf n_2\), the friction-cone checks are:
\[
-\mathbf n_1^\top\mathbf a\geq \frac{1}{\sqrt{1+\mu^2}},
\qquad
\mathbf n_2^\top\mathbf a\geq \frac{1}{\sqrt{1+\mu^2}}
\]
The friction coefficient \(\mu\) determines how much misalignment the contacts tolerate. These checks identify promising geometric pinches; they do not prove that the object will remain held during lifting.

Unlike the older implementation, this version does not use the global outer-shell filter or the additional point-plane filter.

2. Candidate ranking and diversity filtering

The surviving pairs are ranked using a heuristic that prefers contact midpoints near the object’s bounding-box center and closing axes close to horizontal.

This encourages central grasps and discourages grasps near the upper rim—the slipping problem described in your journal. The bounding-box center is only a geometric proxy; the code does not calculate the object’s true center of mass.

The algorithm then removes near-duplicates. If two candidates have nearly the same center and closing direction, it retains one.

This matters because otherwise the planner could spend all its IK attempts on slightly different versions of the same unsuccessful grasp.

3. Constructing the gripper pose

A contact pair specifies the closing direction, but the robot also needs an approach direction.

Using your Fetch gripper convention:
- Gripper y aligns with the closing axis.
- Gripper x points forward along the fingers.
- Gripper z completes the perpendicular coordinate frame.
The algorithm starts with world-down, \(\mathbf d=(0,0,-1)\), and removes its component along the closing axis:
\[
\mathbf x=
\frac{\mathbf d-(\mathbf d^\top\mathbf a)\mathbf a}
{\|\mathbf d-(\mathbf d^\top\mathbf a)\mathbf a\|},
\qquad
\mathbf y=\mathbf a,
\qquad
\mathbf z=\mathbf x\times\mathbf y
\]
This produces a downward approach perpendicular to the closing direction.

For contact midpoint \(\mathbf c\), the gripper-link target is:
\[
\mathbf p_{\text{grasp}}
=
\mathbf c-\ell\mathbf x
\]
Here, \(\ell\) is pad_center_offset: the distance from the gripper-link origin to the useful contact region of the pads.

The pregrasp is farther back:
\[
\mathbf p_{\text{pregrasp}}
=
\mathbf p_{\text{grasp}}-d\mathbf x
\]
For a downward approach, that places it above the object. Correctly calibrating \(\ell\) is important: the fingertip and the pad’s contact center are different locations.

4. Arm-only damped least-squares IK — _ik()

The IK solver searches over the seven arm joints. Base, torso, head, and finger values remain fixed during an IK solve.

At each iteration, it:
1. Computes the gripper pose using forward kinematics.
2. Measures the position and orientation errors.
3. Estimates how each arm joint changes the gripper pose.
4. Calculates a joint adjustment.
5. Accepts an adjustment only if it reduces the error.
The estimated relationship between joint changes and gripper motion is the Jacobian:
\[
\Delta\mathbf x\approx J\Delta\mathbf q
\]
The damped least-squares update is:
\[
\Delta\mathbf q
=
J^\top(JJ^\top+\lambda^2I)^{-1}\mathbf e
\]
Here, \(\mathbf e\) contains the weighted position and rotation errors. Damping \(\lambda\) reduces excessively large updates near configurations where some movements are difficult.

The code estimates the Jacobian by finite differences: perturb one arm joint slightly, recompute forward kinematics, and measure the resulting change.

It also applies joint limits and tries progressively smaller updates when a full update does not improve the result.

A significant change from the old implementation is the rotation error. The new code uses an axis-angle error derived from the relative rotation. A 180° orientation mismatch therefore remains a large error; it cannot accidentally look like zero error.

5. Multiple IK starting configurations — _seeds()

IK is a local search: its result depends on its starting joint configuration.

The planner tries the current arm pose, several predefined arm poses, and random poses within its search limits. It also tests the equivalent grasp orientation obtained by swapping the two jaws while preserving the approach direction.

This gives it several chances to find a useful arm posture.

An unsuccessful IK search does not prove that a pose is unreachable. It means the configured attempts did not find a solution.

6. Collision checking — _valid() and _edge()

Planning happens in a separate PyBullet client containing a robot copy and copied scene collision geometry. Trial joint configurations therefore do not teleport the live robot.

For each tested configuration, the code checks:
- Joint limits.
- Relevant self-collisions.
- Moving robot links against the environment.
- Robot links against the target object.
- During lift planning, the carried object against the environment.
Contact rules depend on the phase. For example, finger–object contact is expected during closure, while a finger intersecting the table remains invalid.

Checking two endpoints is insufficient, so _edge() samples intermediate joint configurations:
\[
\mathbf q(t)=(1-t)\mathbf q_A+t\mathbf q_B,
\qquad 0\leq t\leq1
\]
Every sampled configuration must pass collision checking. Prismatic joints use a finer distance resolution than revolute joints.

These are discrete checks, not a mathematical guarantee against every collision between samples. Missing mesh geometry also falls back to conservative bounding boxes, which can reject feasible movements.

7. Transit planning with RRT-Connect — _transit()

The transit is the movement from the current arm configuration to the pregrasp.

The planner first tries direct joint interpolation. If it is clear, that simple path is sufficient.

If it is blocked, RRT-Connect grows two search trees:
- One rooted at the starting configuration.
- One rooted at the pregrasp configuration.
It samples an arm configuration, extends one tree toward it, then tries to connect the other tree to that extension. Every extension is collision-checked.

When the trees connect, their branches form a complete route around the obstacle.

The search takes place in seven-dimensional joint space: each point represents seven joint angles. It finds a feasible route within its budget; it does not optimize for the shortest or smoothest route.

8. Cartesian approach and lift planning — _cartesian()

Near the object, the gripper should follow a controlled spatial path.

The approach is divided into small Cartesian steps:
\[
\mathbf p_k=
\mathbf p_{\text{pregrasp}}
+
\frac{k}{N}
\left(\mathbf p_{\text{grasp}}-\mathbf p_{\text{pregrasp}}\right)
\]
The desired orientation stays fixed. Each waypoint is solved using the previous waypoint’s joint solution as the next IK seed.

This encourages continuity. The planner also rejects large joint jumps and checks the connecting joint-space movements for collisions.

The lift uses the same method with an upward target. An initial lift check happens during candidate selection. After closure, the lift is replanned using the measured finger aperture and actual object pose.

For collision prediction, the object is treated as moving with the gripper. This is only a planning assumption: the live simulation receives no attachment constraint, so the object can still slip.

9. Per-step execution and physical verification — step()

The execution states are:
State	Behaviour
OPEN	Gradually open the fingers
TRANSIT	Follow the planned route to pregrasp
APPROACH	Follow the approach waypoints
CLOSE	Gradually close and wait for contact on both fingers
LIFT	Follow the replanned upward path
VERIFY	Check sustained height gain and finger contacts
DONE	Hold the successful lift
FAILED	Hold the measured configuration and report why


Each call returns one absolute joint-position target. Your environment advances the simulation afterward.

The command changes by a limited amount each timestep:
\[
q^{\text{command}}_{t+1}
=
q^{\text{command}}_t+
\operatorname{clip}
\left(q^{\text{target}}-q^{\text{command}}_t,-\delta,\delta\right)
\]
Arm and finger joints have separate limits. This addresses the abrupt closure and lifting movements from your journal.

The controller waits for measured joint positions to reach waypoints and uses timeouts to detect stalls. Closure requires sustained contact on both fingers. Lift success requires actual object height gain and continued contact, while changes in the object’s position relative to the gripper detect slipping.

Your solver should enter the carry state only after done becomes true. Reaching the planned arm configuration alone is not considered a successful grasp.