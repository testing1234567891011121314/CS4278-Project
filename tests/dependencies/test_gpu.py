import subprocess
import shutil

ffmpeg = shutil.which("ffmpeg")
if not ffmpeg:
    print("FFmpeg not found on PATH.")
    exit(1)

processes = []
max_sessions = 0

print("Testing max concurrent NVENC sessions...")

try:
    for i in range(1, 16):
        # Dummy FFmpeg NVENC process reading test source
        cmd = [
            ffmpeg, "-y", "-hide_banner", "-loglevel", "error",
            "-f", "lavfi", "-i", "testsrc=size=640x480:rate=30",
            "-c:v", "h264_nvenc", "-preset", "p1",
            "-f", "null", "-"
        ]
        p = subprocess.Popen(cmd)
        processes.append(p)
        
        # Give FFmpeg a moment to initialize the GPU hardware encoder
        import time; time.sleep(0.3)
        
        if p.poll() is not None:
            print(f"FAILED to start session #{i}. Hardware limit reached!")
            break
        
        max_sessions = i
        print(f"Session #{i} started successfully.")

except Exception as e:
    print(f"Error: {e}")

finally:
    # Clean up all spawned test processes
    for p in processes:
        if p.poll() is None:
            p.terminate()

print(f"\nYour GPU supported a maximum of {max_sessions} concurrent NVENC streams.")