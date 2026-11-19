"""Verify the travel pose against the planner oracle at the actual parked
base pose, then sweep candidate travel poses: valid? RRT to pre-grasp OK?"""
import time
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner, RRTConnect, edge_samples

ARM_LINKS = ("upperarm_roll_link", "elbow_flex_link", "forearm_roll_link",
             "wrist_flex_link", "wrist_roll_link", "gripper_link",
             "r_gripper_finger_link", "l_gripper_finger_link")
CURRENT = np.array([0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0])

CANDIDATES = {
    "current[0,-1,0,-2,0,1,0]": [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0],
    "home[0,0,0,0,0,0,0]": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "tuck1[0,-0.3,0,-1.8,0,0.8,0]": [0.0, -0.3, 0.0, -1.8, 0.0, 0.8, 0.0],
    "tuck2[0,-0.5,0,-2.0,0,1.0,0]": [0.0, -0.5, 0.0, -2.0, 0.0, 1.0, 0.0],
    "tuck3[0,-0.7,0,-2.2,0,1.3,0]": [0.0, -0.7, 0.0, -2.2, 0.0, 1.3, 0.0],
    "tuck4[0,-1.0,0,-2.0,0,1.5,0]": [0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.0],
}


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    env.reset()
    for _ in range(250):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    obs = env._get_obs()
    base = np.asarray([obs["qpos"][0] - 2.0, obs["qpos"][1]])
    op = np.asarray(obs["object_pos"], float)
    print(f"base_world={np.round(base,3)} object={np.round(op,3)} "
          f"dist={np.linalg.norm(base-op[:2]):.2f}")

    planner = GraspPlanner(
        env, robot_urdf_path="assets/fetch/fetch.urdf",
        disabled_self_pairs=[(h, a) for h in ("head_pan_link", "head_tilt_link")
                             for a in ARM_LINKS])
    names = {j: planner.info[j][12].decode() for j in planner.info}
    names[-1] = "base"
    planner.obstacles = [p.getBodyUniqueId(i, physicsClientId=planner.client)
                         for i in range(p.getNumBodies(physicsClientId=planner.client))]
    planner.obstacles.remove(planner.robot)

    parked = np.asarray(obs["qpos"], float).copy()
    width = 0.04
    nominal = np.full(2, (width - planner.config.closed_gap) / 2)
    opened = nominal + planner.config.opening_clearance / 2

    def colliding(q):
        planner._set(q)
        out = []
        for body in planner.obstacles:
            for c in p.getClosestPoints(planner.robot, body,
                                        planner.config.collision_margin,
                                        physicsClientId=planner.client):
                if c[3] not in planner.moving_links:
                    continue
                if body == planner.object and c[3] in planner.finger_ids \
                        and c[8] >= -planner.config.contact_penetration:
                    continue
                out.append((body, names.get(c[3], c[3]), round(float(c[8]), 4)))
                break
        for c in p.getClosestPoints(planner.robot, planner.robot,
                                    planner.config.collision_margin,
                                    physicsClientId=planner.client):
            if frozenset((c[3], c[4])) in planner.self_pairs:
                out.append(("self", names.get(c[3], c[3]), names.get(c[4], c[4]),
                            round(float(c[8]), 4)))
        return out

    print("CURRENT travel at park:", colliding(planner._full(parked, CURRENT, opened)))

    # top-down pre-grasp over the object
    grasp_pos = op.copy()
    grasp_pos[2] += 0.01
    quat = Rotation.from_matrix(np.column_stack(
        (np.array([0., 0., -1.]), np.array([0., 1., 0.]),
         np.array([1., 0., 0.])))).as_quat()
    ik = None
    from custom_libraries import grasp as G
    ik = G._ArmIK(planner, parked)
    start = parked[planner.arm_indices]
    lower = planner.lower[planner.arm_indices].copy()
    upper = planner.upper[planner.arm_indices].copy()
    wrap = planner.wrap
    lower[wrap] = np.maximum(lower[wrap], start[wrap] - np.pi)
    upper[wrap] = np.minimum(upper[wrap], start[wrap] + np.pi)
    pre_pos = grasp_pos - planner.config.pregrasp_distance * np.array([0., 0., 1.])
    pre = ik.solve(pre_pos, quat, start, lower, upper,
                   time.monotonic() + 3.0)
    print("pre-grasp IK:", None if pre is None else np.round(pre, 2))

    for label, pose in CANDIDATES.items():
        q7 = np.array(pose, float)
        bad = colliding(planner._full(parked, q7, opened))
        valid = not bad
        transit = None
        if valid and pre is not None:
            rrt = RRTConnect(lower, upper,
                             lambda a: not colliding(planner._full(parked, a, opened)),
                             resolution=planner.config.joint_resolution,
                             step_size=planner.config.rrt_step,
                             max_iterations=planner.config.rrt_iterations)
            transit = rrt.plan(q7, pre, deadline=time.monotonic() + 1.5)
        print(f"{label}: valid={valid} bad={bad[:3] if bad else ''} "
              f"transit={'ok' if transit is not None else 'FAIL'}")
    ik.close()


if __name__ == "__main__":
    main()
