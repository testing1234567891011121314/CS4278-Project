"""Grasping: from the object's point cloud to a lifted, held object.

    candidates   where the gripper could hold the object (R7, R13)
    ik           joint values for a gripper pose (R8)
    arm_planner  a collision-free arm motion for one candidate (R8)
    executor     follows that motion tick by tick and checks the hold (R9 to R12, R22)
"""
