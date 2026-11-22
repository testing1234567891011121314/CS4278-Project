"""Navigation: moving the base to the object, then the held object to the goal.

    grid           occupancy grid from the maze cubes and the table (R14)
    a_star         shortest path on the grid
    parking        where to stand to pick the object
    base_follower  drives the base along a path, tick by tick
    carry          posture, route and driving with a held object (R17)
    reach          brings the object to the goal (R15)
"""
