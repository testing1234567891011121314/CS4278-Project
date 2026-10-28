from time import time
import pybullet as p
import pybullet_data
import numpy as np
import random 
import trimesh 
from pathlib import Path
from maze_utils import generate_maze_map, add_left_room_to_maze, create_maze_urdf
from copy import deepcopy
# from keyboard_control import KeyBoardController
from solver import Solver
from utils import closest_joint_values

class PickNavReachEnv:

    def __init__(self, 
                 seed=0,
                 object_idx=5,
                 ):
        self.set_seed(seed)

        self.pb_physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
        p.setGravity(0,0,-9.8)

        p.configureDebugVisualizer(lightPosition=[0, 0, 10])
        p.configureDebugVisualizer(rgbBackground=[1, 1, 1])        # white background brightens perception

        p.loadURDF("plane.urdf")

        self.seed = seed
        self.object_idx = object_idx
        
        self.action_scale = 0.05
        self.max_force = 2000000
        self.substeps = 5
        self.debug_point_id = None
        self.debug_line_ids = []
        self._load_scene()
        
        # get initial observation
        self.obs = self._get_obs()
        
        self.step_count = 0
        

    def set_seed(self, seed):
        np.random.seed(seed)
        random.seed(seed)
    
    def _load_scene(self):
        self._load_agent()
        self._load_object_table()
        self._load_maze()
        self._load_object_goal()

    def _load_agent(self):
        # Place the Fetch base near the table
        base_pos = [-2, 0.0, 0.0]
        base_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(
            "assets/fetch/fetch.urdf",
            base_pos,
            base_ori,
            useFixedBase=True,
            # flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
        )

        # Collect joint info (skip fixed)
        n_joints = p.getNumJoints(self.robot_id)
        indices = []
        lowers, uppers, ranges, rest = [], [], [], []

        for j in range(n_joints):
            info = p.getJointInfo(self.robot_id, j)
            joint_type = info[2]
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                indices.append(j)
                lowers.append(info[8])
                uppers.append(info[9])
                ranges.append(info[9] - info[8])
                rest.append(info[10])  # joint damping? (PyBullet packs different things; we keep a placeholder)

                p.setJointMotorControl2(
                    self.robot_id, j, p.VELOCITY_CONTROL, force=0.0,
                )

        self.joint_indices = indices
        self.joint_lower = np.array(lowers, dtype=np.float32)
        self.joint_upper = np.array(uppers, dtype=np.float32)
        self.joint_ranges = np.array(ranges, dtype=np.float32)
        self.rest_poses = np.zeros_like(self.joint_lower, dtype=np.float32)
        
        # self.joint_lower[self.joint_upper==-1] = -np.inf
        # self.joint_upper[self.joint_upper==-1] = np.inf
        # self.joint_ranges[self.joint_upper==np.inf] = np.inf
        
        # print("Controllable joints:", len(self.joint_indices))
        # print("Joint Indices:", self.joint_indices)
        # print("Joint Lower:", self.joint_lower)
        # print("Joint Upper:", self.joint_upper)

        # Set an initial configuration
        self.init_qpos = np.clip(
            np.array([0.0] * len(self.joint_indices)),
            self.joint_lower,
            self.joint_upper,
        )
        self._set_qpos(self.init_qpos)

    def _load_object_table(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
        p.loadURDF("table/table.urdf", baseOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase=1)

        # hyperparameters
        ycb_object_dir_path = "./assets/ycb_objects/"
        ycb_objects_paths = sorted(list(Path(ycb_object_dir_path).glob("*")))
        assert 0 <= self.object_idx and self.object_idx < len(ycb_objects_paths), f"object_idx should be in [0, {len(ycb_objects_paths)-1}]"
        object_urdf_path = (ycb_objects_paths[self.object_idx] / "coacd_decomposed_object_one_link.urdf").absolute()
        object_mesh_path = (ycb_objects_paths[self.object_idx] / "textured.obj").absolute()
        self.object_id = p.loadURDF(str(object_urdf_path), basePosition=[0, 0, 1.0], useFixedBase=0)
        self.object_canonical_mesh = trimesh.load(str(object_mesh_path))
        object_canonical_pc, face_indices = trimesh.sample.sample_surface(self.object_canonical_mesh, 1024)
        self.object_canonical_pc = object_canonical_pc.astype(np.float32)  # (1024, 3)
        self.object_canonical_normals = self.object_canonical_mesh.face_normals[face_indices].astype(np.float32)  # (1024, 3), outward normals
    
    def _load_object_goal(self, ):
        self.goal_pos = np.array([self.maze_out_pos_x, self.maze_out_pos_y, 0.9])
        radius = 0.05
        rgba = (0.0, 1.0, 0.0, 0.9)
        vs = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=rgba,
            visualFramePosition=[0, 0, 0],
        )
        self.goal_marker_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=vs,
            basePosition=self.goal_pos,
            baseOrientation=[0, 0, 0, 1],
            useMaximalCoordinates=True,
        )
        p.setCollisionFilterGroupMask(self.goal_marker_id, -1, 0, 0)

    def _load_maze(self):
        # maze hyperparameters
        n_rows = 12
        n_cols = 16
        room_width = 6
        maze_offset = 2.0
        grid_xy = 1.0
        grid_z = 2.0
        # need to randomize
        random_row_in = np.random.randint(1, n_rows-1)
        random_row_out = np.random.randint(1, n_rows-1) 
        in_pos = (random_row_in, 0)
        out_pos = (random_row_out, n_cols-1)
        
        self.maze_out_pos_x = n_cols * grid_xy - grid_xy / 2 + maze_offset
        self.maze_out_pos_y = n_rows / 2 * grid_xy - out_pos[0] * grid_xy - grid_xy / 2

        maze = generate_maze_map(
            n_rows=n_rows, n_cols=n_cols,
            in_pos=in_pos, out_pos=out_pos,
            grid_density=0.8,
            add_x_min_wall=True, add_x_max_wall=True,
            add_y_min_wall=True, add_y_max_wall=True,
            p_greedy=0.6,
            seed=5,
        )

        maze = add_left_room_to_maze(maze, room_width=room_width)

        self.cube_positions = create_maze_urdf(
            maze, 
            urdf_path="./assets/maze.urdf",
            grid_xy=grid_xy, grid_z=grid_z,
            maze_center_x=(n_cols-room_width)/2*grid_xy + maze_offset, 
            maze_center_y=0.0, 
            maze_center_z=0.0,
            box_color=(0.95, 0.95, 0.98, 1.0),
        )

        p.loadURDF("./assets/maze.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)
        
    def visualize_pc_and_normals(self, object_pc, object_normals, visualize_normals=False):
        # remove old debug items
        if self.debug_point_id is not None:
            p.removeUserDebugItem(self.debug_point_id)
        for lid in self.debug_line_ids:
            p.removeUserDebugItem(lid)
        self.debug_line_ids.clear()

        # add new point cloud
        self.debug_point_id = p.addUserDebugPoints(
            pointPositions=object_pc.tolist(),
            pointColorsRGB=[[0, 0, 1]] * object_pc.shape[0],
            pointSize=2.0,
            lifeTime=0
        )

        # add normals, this may slow down the simulation
        if visualize_normals:
            normal_scale = 0.02
            for i in range(0, object_pc.shape[0], 50):  # e.g. subsample
                start = object_pc[i]
                end = object_pc[i] + normal_scale * object_normals[i]
                lid = p.addUserDebugLine(start, end, [1, 0, 0], 1.5, 0)
                self.debug_line_ids.append(lid)

    def step(self, action):
        """
        Apply an action (absolute joint values) and simulate for `substeps`.

        Notes
        -----
        - `action` is interpreted as *target joint positions* in the same order
        as `self.joint_indices`.
        - Targets are wrapped for revolute joints marked by `wrap_mask`
        (here: joints whose `joint_upper == 314`, i.e., 314 rad ~ sentinel for wrap),
        then hard-clipped to joint limits.
        - Position control is used with per-joint gains and a shared max force.

        Parameters
        ----------
        action : array-like, shape (n_dofs,)
            Absolute joint targets for the controllable DOFs.

        Returns
        -------
        obs : dict
            Observation returned by `_get_obs()`.
        info : dict
            Evaluation metrics returned by `evaluate()`.
        """
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        n = len(self.joint_indices)
        if action.size != n:
            raise ValueError(f"Action size {action.size} != controllable dofs {n}")

        # action = np.clip(action, -1.0, 1.0)
        qpos, _, _, _ = self._get_state()
        print(f"===========================Step {self.step_count}===========================")
        self.step_count += 1
        print("current qpos: ", qpos)
        # target = qpos + self.action_scale * action
        # target = np.clip(target, self.joint_lower, self.joint_upper)
        # print("wrap_mask: ", (self.joint_upper == 314))
        # print("action: ", action)
        target = closest_joint_values(action, qpos, wrap_mask=(self.joint_upper == 314))
        # print("target before clip: ", target)
        target = np.clip(target, self.joint_lower, self.joint_upper)
        print("target: ", target)

        # Position control for all controllable joints
        position_gains = [0.3, 0.3, 0.3,  # base movement: xy trans and z rot 
                           0.3, # torso lift 
                           0.3, 0.3, # head pan & tilt
                           0.3, 0.3, # shoulder pan & lift
                           0.3, 0.3, 0.3, 0.3, 0.3, # arm 
                           0.03, 0.03] # gripper
        # position_gains = np.array([0.3] * len(self.joint_indices))
        velocity_gains = np.zeros_like(position_gains) #np.sqrt(np.array(position_gains))
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.joint_indices,
            controlMode=p.POSITION_CONTROL,
            # controlMode=p.PD_CONTROL,
            targetPositions=target.tolist(),
            # targetVelocities=[0.0] * len(self.joint_indices),
            forces=[self.max_force] * len(self.joint_indices),
            # positionGains=[0.3] * len(self.joint_indices),
            positionGains=position_gains,
            # velocityGains=velocity_gains,
        )

        for _ in range(self.substeps):
            p.stepSimulation()
            time.sleep(1./240.)

        obs = self._get_obs()

        info = self.evaluate()

        return obs, info
    
    def reset(self):
        """Reset the simulation and reload world/robot. Returns initial observation."""
        self._set_qpos(self.init_qpos)
        p.resetBasePositionAndOrientation(self.object_id, [0, 0, 5.0], [0, 0, 0, 1])

        return self._get_obs()
    
    def evaluate(self):
        """
        Evaluate distance between the object position and the goal position.
        """
        _, _, object_pos, _ = self._get_state()
        dist_to_goal = np.linalg.norm(object_pos - self.goal_pos)
        success = dist_to_goal < 0.1

        return {
            "dist_to_goal": dist_to_goal,
            "success": success,
        }
    
    def _get_object_obs(self):
        object_pos, object_xyzw = p.getBasePositionAndOrientation(self.object_id)
        object_rot = np.array(p.getMatrixFromQuaternion(object_xyzw)).reshape(3, 3) 
        object_pc = (self.object_canonical_pc @ object_rot.T) + np.array(object_pos, dtype=np.float32).reshape(1, 3)  # (1024, 3)
        object_normals = (self.object_canonical_normals @ object_rot.T)  # (1024, 3), outward normals
        return {
            "object_pc": object_pc,
            "object_normals": object_normals,
        }
    
    def _get_maze_obs(self):
        pass
    
    def _get_obs(self):
        qpos, qvel, object_pos, object_xyzw = self._get_state() # （15，）（15,）（3,）（4，）
        object_obs = self._get_object_obs()

        # self.visualize_pc_and_normals(object_pc, object_normals, visualize_normals=False)
        return {
            "qpos": deepcopy(qpos),
            "qvel": deepcopy(qvel),
            "object_pos": deepcopy(object_pos),
            "object_xyzw": deepcopy(object_xyzw),
            "goal_pos": deepcopy(self.goal_pos),
            "object_pc": deepcopy(object_obs["object_pc"]),
            "object_normals": deepcopy(object_obs["object_normals"]),
            "cube_positions": deepcopy(self.cube_positions[:, :2]),
        }

    def _get_state(self):
        agent_states = p.getJointStates(self.robot_id, self.joint_indices)
        qpos = np.array([s[0] for s in agent_states], dtype=np.float32)
        qvel = np.array([s[1] for s in agent_states], dtype=np.float32)
        
        object_pos, object_xyzw = p.getBasePositionAndOrientation(self.object_id, self.pb_physics_client)
        return qpos, qvel, np.array(object_pos), np.array(object_xyzw)

    def _set_qpos(self, qpos):
        qpos = np.asarray(qpos, dtype=np.float32).reshape(-1)
        if qpos.size != len(self.joint_indices):
            raise ValueError("qpos size mismatch.")
        for idx, q in zip(self.joint_indices, qpos):
            p.resetJointState(self.robot_id, idx, q)

    @property
    def action_size(self):
        return len(self.joint_indices)

    @property
    def obs_size(self):
        # obs dict size
        size_str = "\n"
        for k, v in self.obs.items():
            if isinstance(v, np.ndarray):
                size_str += f"{k}: {type(v)}, {v.shape}\n"
            else:
                size_str += f"{k}: {type(v)}\n"
        return size_str

    @property
    def agent_qpos(self):
        qpos, _, _, _ = self._get_state()
        return qpos.copy()


if __name__ == "__main__":
    env = PickNavReachEnv(seed=42)
    env.reset()
    print(f"Action size: {env.action_size}, Obs size: {env.obs_size}")

    # Set camera angle
    p.resetDebugVisualizerCamera(
        cameraDistance=8.0,              # Distance from the target, in metres
        cameraYaw=45,                    # Horizontal viewing angle, in degrees
        cameraPitch=-60,                 # Downward viewing angle, in degrees
        cameraTargetPosition=[0, 0, 0],   # World position to look at
        physicsClientId=env.pb_physics_client,
    )
    steps_taken = 0 # Track the number of steps taken in the simulation

    solver = Solver(env)
    # keyboard_controller = KeyBoardController(env)
    
    import time
    for i in range (1000):
    # while True:
        # random_action = np.random.uniform(-1.0, 1.0, size=(env.action_size,))
        action = solver.get_action(env)
        obs, info = env.step(action)
        for k, v in info.items():
            print(f"{k}: {v}")
        # p.stepSimulation()
        # time.sleep(1./240.)


        ## below are additional metrics for debugging ##
        steps_taken += 1

        # Follow the measured base position in world coordinates.
        base_x = float(obs["qpos"][0]) - 2.0
        base_y = float(obs["qpos"][1])

        p.resetDebugVisualizerCamera(
            cameraDistance=4.0,
            cameraYaw=45,
            cameraPitch=-60,
            cameraTargetPosition=[base_x, base_y, 0.5],
            physicsClientId=env.pb_physics_client,
        )

# ## Alternate main loop suggested by ChatGPT: has logging ##
# if __name__ == "__main__":
#     import json
#     import shutil
#     import subprocess
#     import time
#     from pathlib import Path

#     ffmpeg = shutil.which("ffmpeg")
#     if ffmpeg is None:
#         raise RuntimeError("FFmpeg is not available on PATH.")

#     env = PickNavReachEnv(seed=42)
#     env.reset()
#     solver = Solver(env)
#     client = env.pb_physics_client

#     p.configureDebugVisualizer(
#         p.COV_ENABLE_GUI, 0, physicsClientId=client
#     )
#     p.setRealTimeSimulation(0, physicsClientId=client)

#     input("Position the camera, then press Enter to record...")

#     width, height = 640, 480
#     camera = p.getDebugVisualizerCamera(physicsClientId=client)
#     view = camera[2]
#     projection = list(camera[3])
#     projection[0] = projection[5] / (width / height)

#     dt = p.getPhysicsEngineParameters(
#         physicsClientId=client
#     )["fixedTimeStep"]
#     step_duration = dt * env.substeps
#     fps = 1.0 / step_duration

#     # Matching filenames for this run.
#     run_name = f"navigation_{time.time_ns()}"
#     output = Path(f"{run_name}.mp4").resolve()
#     log_output = Path(f"{run_name}.jsonl").resolve()

#     def json_value(value):
#         if isinstance(value, np.ndarray):
#             return value.tolist()
#         if isinstance(value, np.generic):
#             return value.item()
#         raise TypeError(f"Cannot serialize {type(value)}")

#     # Cache readable body/link names once, outside the recording loop.
#     body_names = {}
#     link_names = {}

#     for index in range(p.getNumBodies(physicsClientId=client)):
#         body_id = p.getBodyUniqueId(
#             index, physicsClientId=client
#         )
#         base_name, body_name = p.getBodyInfo(
#             body_id, physicsClientId=client
#         )

#         body_names[body_id] = body_name.decode(
#             "utf-8", errors="replace"
#         )
#         link_names[(body_id, -1)] = base_name.decode(
#             "utf-8", errors="replace"
#         )

#         for link in range(
#             p.getNumJoints(body_id, physicsClientId=client)
#         ):
#             joint_info = p.getJointInfo(
#                 body_id, link, physicsClientId=client
#             )
#             link_names[(body_id, link)] = joint_info[12].decode(
#                 "utf-8", errors="replace"
#             )

#     encoder = subprocess.Popen(
#         [
#             ffmpeg, "-hide_banner", "-loglevel", "error", "-n",
#             "-f", "rawvideo",
#             "-pix_fmt", "rgb24",
#             "-s", f"{width}x{height}",
#             "-framerate", str(fps),
#             "-i", "pipe:0",
#             "-an",
#             "-c:v", "libx264",
#             "-preset", "ultrafast",
#             "-crf", "20",
#             "-pix_fmt", "yuv420p",
#             "-movflags", "+faststart",
#             str(output),
#         ],
#         stdin=subprocess.PIPE,
#     )

#     frames = 0

#     try:
#         with log_output.open("w", encoding="utf-8") as log_file:
#             for step in range(1000):
#                 action = solver.get_action(env)
#                 obs, info = env.step(action)

#                 # Contacts after the most recent physics step.
#                 # Keep ground contacts too; do not discard evidence.
#                 contacts = []

#                 for contact in p.getContactPoints(
#                     bodyA=env.robot_id,
#                     physicsClientId=client,
#                 ):
#                     body_b = contact[2]
#                     link_a = contact[3]
#                     link_b = contact[4]

#                     contacts.append({
#                         "robot_link": link_a,
#                         "robot_link_name": link_names.get(
#                             (env.robot_id, link_a), ""
#                         ),
#                         "other_body": body_b,
#                         "other_body_name": body_names.get(body_b, ""),
#                         "other_link": link_b,
#                         "other_link_name": link_names.get(
#                             (body_b, link_b), ""
#                         ),
#                         "position_on_robot": contact[5],
#                         "position_on_other": contact[6],
#                         "normal_on_other": contact[7],
#                         "distance": contact[8],
#                         "normal_force": contact[9],
#                     })

#                 waypoint_index = getattr(
#                     solver, "waypoint_index", None
#                 )
#                 path = getattr(solver, "path", None)
#                 waypoint_xy = None

#                 if (
#                     path is not None
#                     and waypoint_index is not None
#                     and 0 <= waypoint_index < len(path)
#                 ):
#                     waypoint_xy = solver.convert_to_world(
#                         path[waypoint_index]
#                     )

#                 t0 = time.perf_counter()

#                 image = p.getCameraImage(
#                     width,
#                     height,
#                     viewMatrix=view,
#                     projectionMatrix=projection,
#                     renderer=p.ER_BULLET_HARDWARE_OPENGL,
#                     physicsClientId=client,
#                 )

#                 t1 = time.perf_counter()

#                 rgba = np.asarray(
#                     image[2], dtype=np.uint8
#                 ).reshape(height, width, 4)

#                 encoder.stdin.write(
#                     np.ascontiguousarray(rgba[:, :, :3]).tobytes()
#                 )

#                 t2 = time.perf_counter()

#                 # Frame 0 matches the observation after step 0.
#                 record = {
#                     "frame": frames,
#                     "step": step,
#                     "simulation_time": (step + 1) * step_duration,
#                     "video_time": frames / fps,
#                     "action": action,
#                     "qpos": obs["qpos"],
#                     "qvel": obs["qvel"],
#                     "object_pos": obs["object_pos"],
#                     "object_xyzw": obs["object_xyzw"],
#                     "goal_pos": obs["goal_pos"],
#                     "waypoint_index": waypoint_index,
#                     "waypoint_xy": waypoint_xy,
#                     "path_length": (
#                         len(path) if path is not None else None
#                     ),
#                     "contacts": contacts,
#                     "info": info,
#                     "render_seconds": t1 - t0,
#                     "write_seconds": t2 - t1,
#                 }

#                 log_file.write(
#                     json.dumps(record, default=json_value) + "\n"
#                 )
#                 frames += 1

#                 # Periodic flush instead of flushing every frame.
#                 if frames % 48 == 0:
#                     log_file.flush()

#                 if info["success"]:
#                     break

#     except KeyboardInterrupt:
#         print("\nStopping recording...")

#     finally:
#         try:
#             encoder.stdin.close()
#         except BrokenPipeError:
#             pass

#         result = encoder.wait()

#         if result != 0:
#             raise RuntimeError(
#                 f"FFmpeg failed with exit code {result}."
#             )

#         print(f"Saved {frames} frames: {output}")
#         print(f"Matching state/contact log: {log_output}")