"""Our own grasping and navigation code for the Pick-Nav-Reach solver.

    common        Status, Budget
    robot         the robot as read from the simulator
    world         the scene and the object as read from the observation
    planning_sim  the solver's own copy of the scene, for planning
    collision     clearance queries for the planners, answered in that copy
    grasping/     candidates, inverse kinematics, arm planning, execution
    navigation/   occupancy grid, path search, parking, base following, carry, reach
"""
