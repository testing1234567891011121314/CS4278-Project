import numpy as np
import pybullet as p
from queue import PriorityQueue


class Solver:
    def __init__(self, env, robot_radius):
        self.env = env
        self.robot_radius = float(robot_radius)

        if not np.isfinite(self.robot_radius) or self.robot_radius <= 0:
            raise ValueError("robot_radius must be a positive number in metres.")
        self.action = env.agent_qpos # np.zeros(self.env.action_size)
        self.action_scale = np.array([0.01] * env.action_size)
        self.action_scale[:3] = 0.02  # root movement
        self.action_scale[-2:] = 0.005  # gripper
        self.action_upper_bound = env.joint_upper
        self.action_lower_bound = env.joint_lower
        self.turn = 0
        self.forward = 0
        self.yaw = 0
        self.torso_lift = 0
        self.head_pan = 0
        self.head_tilt = 0
        self.shoulder_pan = 0
        self.shoulder_lift = 0
        self.upperarm_roll = 0
        self.elbow_flex = 0
        self.forearm_roll = 0
        self.wrist_flex = 0
        self.wrist_roll = 0
        self.gripper_open = 0

        self.create_map(env) # stores self.occupancy, self.cell_size, self.x_min, self.y_max
        self.inflated_occupancy = self.create_navigable_map() # returns inflated occupancy grid
        self.path = None

    ## This function is called every time step.
    def get_action(self, env):
        obs = env._get_obs()

        # # Joint values -> robot base world XY.
        qpos = obs["qpos"]
        base_xy = (qpos[0] - 2.0, qpos[1])

        # # World XY -> grid (row, col) tuples.
        start = self.convert_to_grid(base_xy)
        goal = self.convert_to_grid(obs["goal_pos"][:2])


        # Plan once initially.
        if self.path is None:
            self.path = self.a_star(start, goal, self.inflated_occupancy)
            self.waypoint_index = 0
            print("Path length:", len(self.path)) # debug
        
        # Reset movement commands every timestep.
        self.forward = 0
        self.turn = 0
        self.yaw = 0

        if self.path:
            base = np.asarray(base_xy, dtype=float)

            # Advance past reached waypoints or passed waypoints on a straight.
            while self.waypoint_index < len(self.path):
                index = self.waypoint_index
                target = self.convert_to_world(self.path[index])
                reached = np.linalg.norm(target - base) <= 0.03

                passed = False
                if 0 < index < len(self.path) - 1:
                    previous = self.convert_to_world(self.path[index - 1])
                    following = self.convert_to_world(self.path[index + 1])

                    incoming = target - previous
                    outgoing = following - target

                    # Only skip passed waypoints within the same straight section.
                    if np.allclose(incoming, outgoing):
                        direction = incoming / np.linalg.norm(incoming)
                        offset = base - target
                        sideways = offset - np.dot(offset, direction) * direction

                        passed = (
                            np.dot(offset, direction) >= 0
                            and np.linalg.norm(sideways) <= 0.03
                        )

                if not (reached or passed):
                    break

                self.waypoint_index += 1

            if self.waypoint_index < len(self.path):
                index = self.waypoint_index
                end_index = index

                # Extend the target along this straight, stopping at the next corner.
                # Look ahead only when already following the same straight section.
                can_look_ahead = False

                if 0 < index < len(self.path) - 1:
                    incoming = (
                        np.asarray(self.path[index])
                        - np.asarray(self.path[index - 1])
                    )
                    outgoing = (
                        np.asarray(self.path[index + 1])
                        - np.asarray(self.path[index])
                    )
                    can_look_ahead = np.array_equal(incoming, outgoing)

                if can_look_ahead:
                    direction = (
                        np.asarray(self.path[index + 1])
                        - np.asarray(self.path[index])
                    )

                    while end_index < len(self.path) - 1:
                        next_direction = (
                            np.asarray(self.path[end_index + 1])
                            - np.asarray(self.path[end_index])
                        )

                        if not np.array_equal(next_direction, direction):
                            break

                        end_index += 1

                target = self.convert_to_world(self.path[end_index])
                error = target - base
                distance = np.linalg.norm(error)

                if distance > 1e-9:
                    direction = error / distance
                    max_distance = min(distance, 1.0)
                    chosen_distance = 0.0
                    minimum_clearance = self.base_clearance(base)

                    # Check the proposed movement at intervals of at most 2 cm.
                    samples = max(1, int(np.ceil(max_distance / 0.02)))

                    for travel in np.linspace(
                        max_distance / samples, max_distance, samples
                    ):
                        point = base + direction * travel
                        row, col = self.convert_to_grid(point)
                        rows, cols = self.inflated_occupancy.shape

                        if not (0 <= row < rows and 0 <= col < cols):
                            break

                        if self.inflated_occupancy[row, col] != 0:
                            break

                        minimum_clearance = min(
                            minimum_clearance,
                            self.base_clearance(point),
                        )

                        if minimum_clearance < 0.15:
                            allowed_step = 0.02
                        elif minimum_clearance < 0.40:
                            allowed_step = 0.10
                        elif minimum_clearance < 0.80:
                            allowed_step = 0.25
                        else:
                            allowed_step = 1.0

                        if travel > allowed_step + 1e-9:
                            break

                        chosen_distance = travel

                    step_xy = direction * chosen_distance

                    print(
                        f"Clearance: {minimum_clearance:.3f} m | "
                        f"Target displacement: {chosen_distance:.3f} m"
                    )

                    self.action[:2] = qpos[:2]
                    self.forward = step_xy[0] / self.action_scale[0]
                    self.turn = step_xy[1] / self.action_scale[1]
        
        ## Concactenate the movement commands into a single action vector and return it.
        unit_action = [self.forward, self.turn, self.yaw, self.torso_lift, self.head_pan, self.head_tilt, self.shoulder_pan, self.shoulder_lift, self.upperarm_roll, self.elbow_flex, self.forearm_roll, self.wrist_flex, self.wrist_roll, self.gripper_open, self.gripper_open]
        print("unit_action: ", unit_action)
        self.action = self.action + self.action_scale * np.array(unit_action)
        return np.clip(self.action, self.action_lower_bound, self.action_upper_bound)


    ### Helper functions for creating a map of the environment and navigating the environment. ###

    ## When initialising the solver, this function is called to create a map of the environment.
    def create_map(self, env):
        print("Creating map of the environment...")
        obs = env._get_obs()
        cube_positions = obs["cube_positions"]

        wall_size = 1.0
        cell_size = 0.1

        # Map boundaries are half a wall beyond the outermost wall centres.
        x_min = cube_positions[:, 0].min() - wall_size / 2
        x_max = cube_positions[:, 0].max() + wall_size / 2
        y_min = cube_positions[:, 1].min() - wall_size / 2
        y_max = cube_positions[:, 1].max() + wall_size / 2

        n_cols = int(np.ceil((x_max - x_min) / cell_size))
        n_rows = int(np.ceil((y_max - y_min) / cell_size))

        occupancy = np.zeros((n_rows, n_cols), dtype=np.uint8)

        for x, y in cube_positions:
            # Wall boundaries expressed in grid coordinates.
            # Rounding removes floating-point noise because these walls
            # align exactly with the 0.1 m grid.
            col_start = int(round((x - wall_size / 2 - x_min) / cell_size))
            col_end = int(round((x + wall_size / 2 - x_min) / cell_size))

            row_start = int(round((y_max - y - wall_size / 2) / cell_size))
            row_end = int(round((y_max - y + wall_size / 2) / cell_size))

            occupancy[row_start:row_end, col_start:col_end] = 1

        # Store the map and its coordinate reference for later navigation.
        self.occupancy = occupancy
        self.cell_size = cell_size
        self.x_min = x_min
        self.y_max = y_max

        self.occupancy = occupancy

        # Temporary: simulator queries for additional obstacles.
        extra_bounds = self.get_extra_obstacle_bounds(cube_positions)
        self.mark_obstacle_bounds(occupancy, extra_bounds)

        print("Map created. Occupancy grid : ", occupancy)

    def create_navigable_map(self):
        # # Derive the clearance radius from the loaded base-link collision bounds.
        safety_margin = 0.05
        robot_radius_cells = int(np.ceil(
            (self.robot_radius + safety_margin) / self.cell_size
        ))

        print("Robot radius:", self.robot_radius)
        print("Inflation cells:", robot_radius_cells)

        structure_size = 2 * robot_radius_cells + 1
        rows, cols = self.occupancy.shape
        padded_occupancy = np.pad(
            self.occupancy.astype(bool),
            robot_radius_cells,
            mode="constant",
        )
        inflated_occupancy = np.zeros((rows, cols), dtype=bool)

        for row_offset in range(structure_size):
            for col_offset in range(structure_size):
                inflated_occupancy |= padded_occupancy[
                    row_offset : row_offset + rows,
                    col_offset : col_offset + cols,
                ]

        return inflated_occupancy.astype(np.uint8)

    def a_star(self, start, goal, occupancy):
        # A* pathfinding algorithm implementation
        rows, cols = occupancy.shape
        for name, cell in [("start", start), ("goal", goal)]:
            row, col = cell

            if not (0 <= row < rows and 0 <= col < cols):
                print(f"{name} is outside the map: {cell}")
                return []

            if occupancy[row, col] != 0:
                print(f"{name} is blocked after inflation: {cell}")
                return []

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while not open_set.empty():
            current = open_set.get()[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current, rows, cols):
                if occupancy[neighbor] == 1:
                    continue  # Skip occupied cells

                tentative_g_score = g_score[current] + 1  # Assume cost between neighbors is 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

        return []  # Return empty path if no path found

    def heuristic(self, start, goal):
        # Use Manhattan distance as heuristic
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def get_neighbors(self, cell, rows, cols):
        row, col = cell
        neighbors = []
        for d_row, d_col in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_row, new_col = row + d_row, col + d_col
            if 0 <= new_row < rows and 0 <= new_col < cols:
                neighbors.append((new_row, new_col))
        return neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path

    def convert_to_grid(self, position):
        col = int(np.floor(
            (position[0] - self.x_min) / self.cell_size
        ))
        row = int(np.floor(
            (self.y_max - position[1]) / self.cell_size
        ))
        return row, col

    def convert_to_world(self, cell):
        row, col = cell
        x = self.x_min + (col + 0.5) * self.cell_size
        y = self.y_max - (row + 0.5) * self.cell_size
        return np.array([x, y])

    def debug_env(self, env):
        ## These are the provided @properties.
        print("Action size = ", env.action_size)
        print("Obs size = ", env.obs_size)
        print("Agent position = ", env.agent_qpos)
        ## below are additional info
        obs = env._get_obs()
        print("Object position = ", obs["object_pos"])
        print("Object xyzw = ", obs["object_xyzw"])
        print("Goal position = ", obs["goal_pos"])
        print("Object point cloud = ", obs["object_pc"])
        print("Object normals = ", obs["object_normals"])
        print("Wall position = ", obs["cube_positions"])

    ### Helper functions for querying pybullet directly to find the table. ###
    def get_extra_obstacle_bounds(self, cube_positions):
        """Read additional obstacle bounds without changing the simulation."""
        client = self.env.pb_physics_client
        obstacles = []

        excluded = {
            self.env.robot_id,
            self.env.object_id,       # Grasp target: handled separately.
            self.env.goal_marker_id,
        }

        for index in range(p.getNumBodies(physicsClientId=client)):
            body_id = p.getBodyUniqueId(
                index, physicsClientId=client
            )

            if body_id in excluded:
                continue

            link_bounds = []

            for link in range(
                -1,
                p.getNumJoints(body_id, physicsClientId=client),
            ):
                shapes = p.getCollisionShapeData(
                    body_id, link, physicsClientId=client
                )

                if not shapes:
                    continue

                lower, upper = p.getAABB(
                    body_id, link, physicsClientId=client
                )
                link_bounds.append((
                    np.asarray(lower),
                    np.asarray(upper),
                ))

            if not link_bounds:
                continue

            body_min = np.min([b[0] for b in link_bounds], axis=0)
            body_max = np.max([b[1] for b in link_bounds], axis=0)

            # This environment's ground surface is at z = 0.
            if body_max[2] <= 0.01:
                continue

            # Recognise the merged maze by its bounds enclosing every
            # supplied wall centre. Its walls are already mapped separately.
            contains_walls = np.all(
                (cube_positions >= body_min[:2] - 1e-3)
                & (cube_positions <= body_max[:2] + 1e-3)
            )

            if contains_walls:
                continue

            obstacles.extend(link_bounds)

        return obstacles

    def mark_obstacle_bounds(self, occupancy, obstacle_bounds):
        rows, cols = occupancy.shape

        for lower, upper in obstacle_bounds:
            col_start = int(np.floor(
                (lower[0] - self.x_min) / self.cell_size
            ))
            col_end = int(np.ceil(
                (upper[0] - self.x_min) / self.cell_size
            ))

            row_start = int(np.floor(
                (self.y_max - upper[1]) / self.cell_size
            ))
            row_end = int(np.ceil(
                (self.y_max - lower[1]) / self.cell_size
            ))

            col_start = max(0, min(cols, col_start))
            col_end = max(0, min(cols, col_end))
            row_start = max(0, min(rows, row_start))
            row_end = max(0, min(rows, row_end))

            occupancy[row_start:row_end, col_start:col_end] = 1
    def base_clearance(self, position):
        # Cache occupied-cell centres once; includes any mapped table.
        if not hasattr(self, "_obstacle_centres"):
            rows, cols = np.nonzero(self.occupancy)

            self._obstacle_centres = np.column_stack((
                self.x_min + (cols + 0.5) * self.cell_size,
                self.y_max - (rows + 0.5) * self.cell_size,
            ))

        if len(self._obstacle_centres) == 0:
            return float("inf")

        distances = np.linalg.norm(
            self._obstacle_centres - np.asarray(position),
            axis=1,
        )

        # Subtract cell extent and robot radius for conservative clearance.
        half_cell_diagonal = self.cell_size / np.sqrt(2)

        return float(
            distances.min() - half_cell_diagonal - self.robot_radius
        )