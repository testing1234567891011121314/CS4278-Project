"""Unit test for AntipodalGraspPlanner.plan_arm_transit + collision oracle.

Headless (p.DIRECT): loads Fetch + table + maze-like box, builds the
oracle, plans from the stow config to a reachable pre-grasp-like goal.
Checks: endpoints, continuity, and that the oracle itself rejects a
config with the arm driven into the table.
"""
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
from pathlib import Path

from custom_libraries.antipodal import AntipodalGraspPlanner

HERE = Path(__file__).parent


def main():
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    plane = p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        str(HERE / "assets/fetch/fetch.urdf"),
        [-2, 0.0, 0.0],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True,
        physicsClientId=cid,
    )
    table = p.loadURDF("table/table.urdf",
                       baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2]),
                       useFixedBase=1, physicsClientId=cid)
    # Box wall between robot and table (maze-proxy)
    vs = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 1.0])
    cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 1.0])
    wall = p.createMultiBody(0, cs, vs, basePosition=[-0.7, 1.5, 1.0])  # off to the side

    joint_ids = {}
    for j in range(p.getNumJoints(robot_id, physicsClientId=cid)):
        info = p.getJointInfo(robot_id, j, physicsClientId=cid)
        joint_ids[info[1].decode()] = j
    arm_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                 "upperarm_roll_joint", "elbow_flex_joint",
                 "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    arm_ids = [joint_ids[n] for n in arm_names]
    ee_link = [j for j in range(p.getNumJoints(robot_id, physicsClientId=cid))
               if p.getJointInfo(robot_id, j, physicsClientId=cid)[12].decode()
               == "gripper_link"][0]

    lower = np.clip(np.array([p.getJointInfo(robot_id, j, physicsClientId=cid)[8]
                              for j in arm_ids], float), -np.pi, np.pi)
    upper = np.clip(np.array([p.getJointInfo(robot_id, j, physicsClientId=cid)[9]
                              for j in arm_ids], float), -np.pi, np.pi)

    planner = AntipodalGraspPlanner(physics_client_id=cid)
    start_q = np.array([0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0])  # travel pose
    for j, v in zip(arm_ids, start_q):
        p.resetJointState(robot_id, j, float(v), physicsClientId=cid)

    is_state_valid = planner.make_collision_checker(robot_id, arm_ids, margin=0.01)

    # Oracle sanity: start should be valid.
    assert is_state_valid(start_q), "travel pose flagged invalid (too-tight margin?)"
    # Reachable goal: FK sweep around the top-down basin from the journal
    goal_q = np.array([-0.4, -0.62, 1.13, 0.91, -0.65, 1.88, -1.74])
    for j, v in zip(arm_ids, goal_q):
        p.resetJointState(robot_id, j, float(v), physicsClientId=cid)
    st = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True,
                        physicsClientId=cid)
    goal_pos = np.array(st[4])
    print("goal ee pos:", np.round(goal_pos, 3))
    print("goal valid?", is_state_valid(goal_q))

    t0 = time.time()
    path = planner.plan_arm_transit(
        robot_id, ee_link, arm_ids, start_q, goal_q,
        is_state_valid, lower, upper, time_budget=8.0)
    dt = time.time() - t0
    print(f"plan time {dt:.2f}s")

    # Oracle MUST reject: drop a box exactly on the goal gripper pose.
    block = p.createMultiBody(
        0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06]),
        p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06]),
        basePosition=goal_pos.tolist(), physicsClientId=cid)
    is_state_valid2 = planner.make_collision_checker(robot_id, arm_ids,
                                                     margin=0.01)
    assert not is_state_valid2(goal_q), \
        "oracle failed to flag gripper inside a box"
    blocked = planner.plan_arm_transit(
        robot_id, ee_link, arm_ids, start_q, goal_q,
        is_state_valid2, lower, upper, time_budget=3.0)
    assert blocked is None, "RRT accepted a plan ending inside an obstacle"
    p.removeBody(block, physicsClientId=cid)
    print("PASS: oracle rejects gripper-in-obstacle")

    assert path is not None, "RRT failed to find a path"
    path = [np.asarray(q, float) for q in path]
    assert np.allclose(path[0], np.clip(start_q, lower, upper), atol=1e-6)
    assert np.allclose(path[-1], np.clip(goal_q, lower, upper), atol=1e-6)
    steps = [float(np.linalg.norm(b - a)) for a, b in zip(path, path[1:])]
    assert max(steps) <= 0.5, f"step too large: {max(steps)}"
    for q in path:
        assert np.all(q >= lower - 1e-6) and np.all(q <= upper + 1e-6)
    print(f"PASS: {len(path)} configs, max step {max(steps):.3f} rad")

    p.disconnect(physicsClientId=cid)


if __name__ == "__main__":
    sys.exit(main())
