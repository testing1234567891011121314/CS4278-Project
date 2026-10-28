import pkgutil
import numpy as np
import pybullet as p
import pybullet_data

# 1. Connect in Headless DIRECT Mode
cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)

# 2. Try Loading the EGL Hardware Plugin
egl_plugin_id = -1
egl = pkgutil.get_loader("eglRenderer")
if egl:
    egl_plugin_id = p.loadPlugin(
        egl.get_filename(), "_eglRendererPlugin", physicsClientId=cid
    )

print(f"EGL Plugin Load Result ID: {egl_plugin_id}")

if egl_plugin_id < 0:
    print("❌ EGL Plugin failed to load. PyBullet is using CPU TinyRenderer.")
else:
    print("✅ EGL Hardware Plugin successfully loaded into PyBullet!")

# 3. Load a 3D Model and Render via ER_BULLET_HARDWARE_OPENGL
p.loadURDF("r2d2.urdf", [0, 0, 1], physicsClientId=cid)
view_matrix = p.computeViewMatrixFromYawPitchRoll(
    [0, 0, 1], 3.0, 45, -30, 0, 2, physicsClientId=cid
)
proj_matrix = p.computeProjectionMatrixFOV(
    60, 640 / 480, 0.1, 100, physicsClientId=cid
)

img = p.getCameraImage(
    640,
    480,
    viewMatrix=view_matrix,
    projectionMatrix=proj_matrix,
    renderer=p.ER_BULLET_HARDWARE_OPENGL,
    physicsClientId=cid,
)

rgba = np.array(img[2], dtype=np.uint8).reshape(480, 640, 4)

# 4. Analyze Pixel Content
mean_val = rgba[:, :, :3].mean()
std_val = rgba[:, :, :3].std()

print(f"Frame Pixel RGB Mean: {mean_val:.2f} (Solid white = 255.0)")
print(f"Frame Pixel Standard Deviation: {std_val:.2f}")

if mean_val < 250.0 and std_val > 5.0:
    print(
        "🎉 SUCCESS: Headless Hardware GPU Rendering (EGL) is working perfectly!"
    )
else:
    print("⚠️ WARNING: Frame is solid white/blank. EGL context did not bind.")

p.disconnect(cid)