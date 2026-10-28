import multiprocessing as mp
import numpy as np
import pybullet as p
import pybullet_data

def worker_render_task(worker_id):
    try:
        # Test DIRECT (headless) vs GUI mode
        client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        p.loadURDF("plane.urdf", physicsClientId=client)
        
        view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 5.0, 45, -55, 0, 2, physicsClientId=client)
        proj = p.computeProjectionMatrixFOV(60.0, 640/480, 0.1, 100.0, physicsClientId=client)
        
        # Render 50 frames
        for _ in range(50):
            img = p.getCameraImage(640, 480, viewMatrix=view, projectionMatrix=proj, physicsClientId=client)
            rgba = np.asarray(img[2], dtype=np.uint8)
            assert rgba.shape == (480, 640, 4)
            
        p.disconnect(client)
        print(f"Worker {worker_id}: ✅ Successfully rendered 50 frames in PyBullet.")
        return True
    except Exception as e:
        print(f"Worker {worker_id}: ❌ PyBullet rendering error: {e}")
        return False

if __name__ == "__main__":
    mp.freeze_support()
    print("Testing 15 concurrent PyBullet rendering workers...")
    with mp.Pool(processes=15) as pool:
        results = pool.map(worker_render_task, range(15))
    
    if all(results):
        print("✅ SUCCESS: PyBullet multi-process rendering is stable.")
    else:
        print("❌ FAILED: PyBullet process/rendering context issue detected.")