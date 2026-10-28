import json
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import pybullet as p


def record_navigation(
    env, solver, encoder_args, label, max_steps=1000,
    *,
    interactive=True,
):
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("FFmpeg is not available on PATH.")

    client = env.pb_physics_client
    if interactive:
        input("Position the camera, then press Enter to record...")

    width, height = 1920, 1080
    camera = p.getDebugVisualizerCamera(physicsClientId=client)
    view = camera[2]

    # Safe check for DIRECT (headless) mode
    if camera is not None and len(camera) >= 4:
        projection = list(camera[3])
        projection[0] = projection[5] / (width / height)
    else:
        # Fallback projection matrix for p.DIRECT mode
        # projection = p.computeProjectionMatrixFOV(
        #     fov=60.0,
        #     aspect=float(width) / float(height),
        #     nearVal=0.1,
        #     farVal=100.0,
        #     physicsClientId=client,
        # )
        projection = p.computeProjectionMatrixFOV(
            fov=60.0,
            aspect=width / height,
            nearVal=0.1,
            farVal=100.0,
        )

    dt = p.getPhysicsEngineParameters(
        physicsClientId=client
    )["fixedTimeStep"]

    step_duration = dt * env.substeps
    fps = 1.0 / step_duration

    run_name = f"navigation_{label}_{time.time_ns()}"
    output = Path(f"{run_name}.mp4").resolve()
    log_output = Path(f"{run_name}.jsonl").resolve()
    error_output = Path(f"{run_name}_ffmpeg.log").resolve()

    def json_value(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f"Cannot serialize {type(value)}")

    # Resolve body IDs and link names dynamically.
    body_names = {}
    link_names = {}

    for index in range(p.getNumBodies(physicsClientId=client)):
        body_id = p.getBodyUniqueId(index, physicsClientId=client)
        base_name, body_name = p.getBodyInfo(
            body_id, physicsClientId=client
        )

        body_names[body_id] = body_name.decode(
            "utf-8", errors="replace"
        )
        link_names[(body_id, -1)] = base_name.decode(
            "utf-8", errors="replace"
        )

        for link in range(
            p.getNumJoints(body_id, physicsClientId=client)
        ):
            joint_info = p.getJointInfo(
                body_id, link, physicsClientId=client
            )
            link_names[(body_id, link)] = joint_info[12].decode(
                "utf-8", errors="replace"
            )

    ## Original command does about 3 physics steps/frames per second
    # command = [
    #     ffmpeg,
    #     "-hide_banner", "-loglevel", "error", "-n",
    #     "-f", "rawvideo",
    #     "-pix_fmt", "rgb24",
    #     "-s", f"{width}x{height}",
    #     "-framerate", str(fps),
    #     "-i", "pipe:0",
    #     "-an",
    #     *encoder_args,
    #     "-pix_fmt", "yuv420p",
    #     "-movflags", "+faststart",
    #     str(output),
    # ]

    command = [
        ffmpeg,
        "-hide_banner", "-loglevel", "error", "-n",
        "-f", "rawvideo",
        "-pix_fmt", "rgba",
        "-s", f"{width}x{height}",
        "-framerate", str(fps),
        "-i", "pipe:0",
        "-an",
        *encoder_args,
        "-pix_fmt", "yuv420p",
        "-movflags", "+faststart",
        str(output),
    ]

    frames = 0
    broken_pipe = False

    # Write FFmpeg errors to disk to avoid a blocked stderr pipe.
    with error_output.open("wb") as error_file:
        encoder = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stderr=error_file,
        )

        try:
            with log_output.open("w", encoding="utf-8") as log_file:
                for step in range(max_steps):
                    loop_start = time.perf_counter()

                    t0 = time.perf_counter()
                    action = solver.get_action(env)
                    action_seconds = time.perf_counter() - t0

                    t0 = time.perf_counter()
                    obs, info = env.step(action)
                    physics_seconds = time.perf_counter() - t0

                    t0 = time.perf_counter()
                    contacts = []

                    # Contacts from the latest physics substep.
                    for contact in p.getContactPoints(
                        bodyA=env.robot_id,
                        physicsClientId=client,
                    ):
                        body_b = contact[2]
                        link_a = contact[3]
                        link_b = contact[4]

                        contacts.append({
                            "robot_link": link_a,
                            "robot_link_name": link_names.get(
                                (env.robot_id, link_a), ""
                            ),
                            "other_body": body_b,
                            "other_body_name": body_names.get(body_b, ""),
                            "other_link": link_b,
                            "other_link_name": link_names.get(
                                (body_b, link_b), ""
                            ),
                            "position_on_robot": contact[5],
                            "position_on_other": contact[6],
                            "normal_on_other": contact[7],
                            "distance": contact[8],
                            "normal_force": contact[9],
                        })

                    contact_seconds = time.perf_counter() - t0

                    waypoint_index = getattr(
                        solver, "waypoint_index", None
                    )
                    path = getattr(solver, "path", None)
                    waypoint_xy = None

                    if (
                        path is not None
                        and waypoint_index is not None
                        and 0 <= waypoint_index < len(path)
                    ):
                        waypoint_xy = solver.convert_to_world(
                            path[waypoint_index]
                        )

                    t0 = time.perf_counter()

                    # Follow the measured robot base position.
                    base_x = float(obs["qpos"][0]) - 2.0
                    base_y = float(obs["qpos"][1])

                    view = p.computeViewMatrixFromYawPitchRoll(
                        cameraTargetPosition=[base_x, base_y, 0.7],
                        distance=5.0,
                        yaw=45.0,
                        pitch=-55.0,
                        roll=0.0,
                        upAxisIndex=2,
                    )

                    image = p.getCameraImage(
                        width,
                        height,
                        viewMatrix=view,
                        projectionMatrix=projection,
                        renderer=p.ER_BULLET_HARDWARE_OPENGL,
                        flags=p.ER_NO_SEGMENTATION_MASK,
                        physicsClientId=client,
                    )

                    # debug
                    if step == 0:
                        print(type(image[2]), getattr(image[2], "shape", None))

                    render_seconds = time.perf_counter() - t0

                    t0 = time.perf_counter()
                    rgba = np.asarray(
                        image[2], dtype=np.uint8
                    ).reshape(height, width, 4)

                    ## Replace original conversion with direct tobytes() call
                    # rgb_bytes = np.ascontiguousarray(
                    #     rgba[:, :, :3]
                    # ).tobytes()
                    rgba_bytes = rgba.tobytes()

                    convert_seconds = time.perf_counter() - t0

                    t0 = time.perf_counter()
                    encoder.stdin.write(rgba_bytes)
                    write_seconds = time.perf_counter() - t0

                    record = {
                        "frame": frames,
                        "step": step,
                        "simulation_time": (step + 1) * step_duration,
                        "video_time": frames / fps,
                        "action": action,
                        "qpos": obs["qpos"],
                        "qvel": obs["qvel"],
                        "object_pos": obs["object_pos"],
                        "object_xyzw": obs["object_xyzw"],
                        "goal_pos": obs["goal_pos"],
                        "waypoint_index": waypoint_index,
                        "waypoint_xy": waypoint_xy,
                        "path_length": (
                            len(path) if path is not None else None
                        ),
                        "contacts": contacts,
                        "info": info,
                        "action_seconds": action_seconds,
                        "physics_seconds": physics_seconds,
                        "contact_seconds": contact_seconds,
                        "render_seconds": render_seconds,
                        "convert_seconds": convert_seconds,
                        "write_seconds": write_seconds,
                    }

                    t0 = time.perf_counter()
                    log_file.write(
                        json.dumps(record, default=json_value) + "\n"
                    )
                    frames += 1

                    if frames % 48 == 0:
                        log_file.flush()

                    log_seconds = time.perf_counter() - t0
                    total_seconds = time.perf_counter() - loop_start

                    if step % 20 == 0:
                        print(
                            f"[{label}] Step {step}: "
                            f"solver={action_seconds:.3f}s | "
                            f"env.step={physics_seconds:.3f}s | "
                            f"contacts={contact_seconds:.3f}s | "
                            f"render={render_seconds:.3f}s | "
                            f"convert={convert_seconds:.3f}s | "
                            f"write={write_seconds:.3f}s | "
                            f"log={log_seconds:.3f}s | "
                            f"total={total_seconds:.3f}s"
                        )

                    if info.get("success", False):
                        print("Task succeeded.")
                        break

        except KeyboardInterrupt:
            print("\nStopping recording...")

        except BrokenPipeError:
            broken_pipe = True

        finally:
            try:
                encoder.stdin.close()
            except BrokenPipeError:
                broken_pipe = True

            result = encoder.wait()

    if broken_pipe or result != 0:
        details = error_output.read_text(
            encoding="utf-8", errors="replace"
        )
        raise RuntimeError(
            f"FFmpeg failed (exit code {result}).\n"
            f"{details}\nError log: {error_output}"
        )

    print(f"Saved {frames} frames: {output}")
    print(f"Matching state/contact log: {log_output}")