# Fixes for Antipodal Grasp Generation

`AntipodalGraspPlanner` correctly generates geometric antipodal grasp candidates from a point cloud, but it stops there: it never solves inverse kinematics, never considers the object's mass or center of mass, and never adjusts grip force during the lift — all three of which the module's own docstring calls for. Five fixes are detailed below, ordered by how early they block the stated goals.

## 1. Missing IK solve step

**Problem.** `sample_antipodal_grasps` builds a target position and quaternion orientation for each candidate grasp, but never converts that pose into joint angles. There is no `calculateInverseKinematics` call, no Jacobian, nothing that touches the robot's joints. The docstring's first line ("should implement quaternion based inverse kinematics") is not yet true of the code — what exists is grasp *pose generation*, which IK must still consume.

**Fix.** Add a reachability-check step that runs IK on each ranked candidate and keeps the first one that converges within joint limits:

```python
def solve_ik(self, robot_id, ee_link_index, target_pos, target_orn, joint_limits=None):
    joint_angles = p.calculateInverseKinematics(
        robot_id, ee_link_index, target_pos, target_orn,
        maxNumIterations=100, residualThreshold=1e-4
    )
    if joint_limits and not self._within_limits(joint_angles, joint_limits):
        return None
    return joint_angles

def select_feasible_grasp(self, robot_id, ee_link_index, candidates):
    for pos, quat in candidates:
        joints = self.solve_ik(robot_id, ee_link_index, pos, quat)
        if joints is not None:
            return pos, quat, joints
    return None
```

PyBullet's built-in solver is damped-least-squares based, so it inherits the same near-singularity behavior covered earlier: candidates close to a kinematic singularity may converge to a large, physically implausible joint configuration even when `residualThreshold` is met — worth sanity-checking the returned angles against a max-step bound before accepting them.

## 2. No mass/COM/torque awareness in grasp ranking

**Problem.** `valid_grasps.sort(key=lambda g: -g[0][2])` ranks candidates purely by z-height — a heuristic for "prefer top-down." No object mass, center of mass, or inertia enters the scoring. A grasp at an object's geometric centroid passes the antipodal/friction-cone test fine even when the true center of mass is far off-center (e.g., a hammer gripped between handle and head) — the lever arm from contact to true COM creates a torque the geometric test never evaluates, and the current sort would rate that grasp as good as any other at the same height.

**Fix.** Add a wrench-space stability filter between candidate generation and ranking. Pull mass and COM from PyBullet directly:

```python
def gravity_wrench(self, object_id, p1, p2):
    mass, _, com, *_ = p.getDynamicsInfo(object_id, -1)[:3], p.getBasePositionAndOrientation(object_id)[0]
    contact_centroid = (p1 + p2) / 2.0
    r = np.array(com) - contact_centroid
    force = np.array([0, 0, -mass * 9.81])
    torque = np.cross(r, force)
    return force, torque

def grasp_quality(self, p1, p2, n1, n2, mu, object_id):
    force, torque = self.gravity_wrench(object_id, p1, p2)
    # Ferrari-Canny style check: can the two friction cones supply enough
    # normal/tangential force at p1, p2 to balance (force, torque)?
    return self._wrench_in_grasp_space(p1, p2, n1, n2, mu, force, torque)
```

A full Ferrari-Canny implementation (discretized friction cone rays, convex hull of achievable wrenches, point-in-hull test) is more robust; a cheap first pass — comparing required torque magnitude to an estimated maximum frictional torque at the two contacts — catches the worst offenders (badly off-center grasps) before the more expensive IK step runs on them. Sort candidates by this quality score instead of raw z-height, using height only as a tiebreaker.

## 3. No per-step grip force adjustment during lift

**Problem.** The docstring requires accounting for "torque while lifting" and adjusting grip strength at each step — but `sample_antipodal_grasps` runs once, before execution, and nothing in the class revisits the grasp afterward. There is no execution-time loop at all.

**Fix.** This needs a genuinely new method, called once per control step during the lift, not a change to the existing grasp-selection function. At each step: read the arm's current/planned acceleration, compute the resulting inertial disturbance on the object, add it to the static gravity wrench, and increase gripper force if the combined wrench is at risk of leaving the grasp's achievable wrench space (i.e., before slip occurs, not after it's detected):

```python
def required_grip_force(self, object_id, linear_accel, angular_accel, com, contact_centroid, mu):
    mass = p.getDynamicsInfo(object_id, -1)[0]
    gravity_force = np.array([0, 0, -mass * 9.81])
    inertial_force = -mass * np.array(linear_accel)
    total_force = gravity_force + inertial_force
    # minimum normal force so tangential component stays inside the friction cone:
    normal_component = np.dot(total_force, self._grasp_axis)
    tangential_mag = np.linalg.norm(total_force - normal_component * self._grasp_axis)
    return max(abs(normal_component), tangential_mag / mu)

def adjust_grip_during_lift(self, gripper_id, object_id, planned_trajectory_step):
    accel = planned_trajectory_step.linear_accel
    ang_accel = planned_trajectory_step.angular_accel
    needed_force = self.required_grip_force(object_id, accel, ang_accel, self.com, self.contact_centroid, self.mu)
    p.setJointMotorControl2(gripper_id, self.gripper_joint, p.TORQUE_CONTROL, force=needed_force)
```

This should be called from whatever drives the lift trajectory (once per control tick, or at least once per planned waypoint), not from the grasp planner itself — the planner's job stops at selecting a feasible, high-quality grasp (Fixes 1–2); grip-force adaptation belongs to the execution loop that follows it.

## 4. O(n²) pairwise candidate search performance

**Problem.** The nested Python `for i / for j` loop checks all \~500,000 pairs for `num_samples=500` points — pure interpreter overhead, and it only grows worse at the 1024-point scale mentioned in discussion. This is a real bottleneck, not a fundamental limit of the antipodal test itself.

**Fix, two complementary changes:**

1. **Vectorize the existing check with NumPy broadcasting** instead of the nested loop — compute all pairwise distances and friction-cone dot products as array operations.
2. **Exploit the "grippers are strictly opposite each other" structure directly**, rather than checking every pair: for each point `pᵢ` with normal `n̂ᵢ`, cast a ray from `pᵢ` in the direction `−n̂ᵢ` (straight through the object) and only test the points it actually hits as antipodal candidates. This mirrors what production grasp-detection pipelines (e.g., Grasp Pose Detection / GPG) do, and collapses the search from O(n²) all-pairs to roughly O(n log n) with a spatial index (octree/kd-tree) resolving each ray.

```python
def vectorized_antipodal_check(self, points, normals, max_width, min_cos):
    diffs = points[:, None, :] - points[None, :, :]        # (n, n, 3)
    dists = np.linalg.norm(diffs, axis=-1)
    valid_dist = (dists < max_width) & (dists > 0.01)
    d12 = diffs / np.where(dists[..., None] == 0, 1, dists[..., None])
    cos1 = np.einsum('ijk,ik->ij', d12, normals)
    cos2 = np.einsum('ijk,ik->ij', -d12, normals[:, None, :].squeeze())
    valid = valid_dist & (cos1 > min_cos) & (cos2 > min_cos)
    return np.argwhere(np.triu(valid, k=1))
```

A mass-spring physics graph was also considered for this stage, but is the wrong tool here: it adds an iterative, timestep-sensitive solver and an arbitrary stiffness constant with no physical meaning for rigid YCB objects, in place of the exact closed-form friction-cone test that already exists. It would be the right tool only for genuinely deformable objects (foam, fabric), where simulated squeeze response is itself part of what determines grasp stability — a different problem than the current rigid-body grasp search.

## 5. Incomplete point cloud sampling coverage

**Problem.** `get_object_point_cloud` only casts rays from a horizontal ring around the AABB center, at 4 fixed heights. This is effectively cylindrical sampling and will miss surface points on top/bottom caps — e.g. the poles of a sphere, or a flat top face of a box — silently excluding legitimate top-down grasp candidates depending on object shape.

**Fix.** Add a second ray-casting pass from above and below the object, directed straight down/up through the AABB, on a grid over x/y rather than only around the perimeter:

```python
def _sample_cap_rays(self, object_id, aabb_min, aabb_max, grid_n=8):
    points, normals = [], []
    xs = np.linspace(aabb_min[0], aabb_max[0], grid_n)
    ys = np.linspace(aabb_min[1], aabb_max[1], grid_n)
    for x in xs:
        for y in ys:
            for z_start, z_end in [(aabb_max[2] + 0.05, aabb_min[2]),
                                     (aabb_min[2] - 0.05, aabb_max[2])]:
                res = p.rayTest([x, y, z_start], [x, y, z_end])
                if res and res[0][0] == object_id:
                    points.append(np.array(res[0][3]))
                    normals.append(np.array(res[0][4]))
    return points, normals
```

Call this alongside the existing ring sampling in `get_object_point_cloud` and concatenate the results before passing to `sample_antipodal_grasps`.

## 6. Summary

| # | Fix | Blocks | Effort |
| --- | --- | --- | --- |
| 1 | Add IK reachability check per candidate | Grasps are never actually executable | Small — wraps `calculateInverseKinematics` |
| 2 | Rank by gravity/wrench-space stability, not z-height | Off-center-mass grasps pass as "good" | Medium — needs mass/COM query + wrench check |
| 3 | Add per-step grip-force control loop during lift | Docstring's core requirement; currently absent entirely | Large — new execution-time component |
| 4 | Vectorize antipodal search + ray-based pruning | Runtime at 1024-point scale | Medium — NumPy rewrite, optional ray-cast pruning |
| 5 | Add top/bottom cap ray sampling | Missed grasp candidates on flat/curved caps | Small — one additional sampling pass |

Recommended order: 1 and 5 first (small, unblock correctness), then 4 (unblocks working at full point-cloud scale), then 2 and 3 (the two pieces the docstring explicitly asks for but that require the most new design work).
