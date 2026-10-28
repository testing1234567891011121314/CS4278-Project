import shutil
import subprocess
import numpy as np

ffmpeg = shutil.which("ffmpeg")
if not ffmpeg:
    print("❌ FFmpeg is not installed or not on PATH!")
    exit(1)

width, height = 1920, 1080
fps = 24.0

# Exact encoder flags used in trial_worker.py and recording.py
command = [
    ffmpeg,
    "-hide_banner", "-loglevel", "verbose", "-y",
    "-f", "rawvideo",
    "-pix_fmt", "rgba",
    "-s", f"{width}x{height}",
    "-framerate", str(fps),
    "-i", "pipe:0",
    "-an",
    "-c:v", "h264_nvenc", "-preset", "p1",  # Test NVENC (or change to 'libx264' for CPU)
    "-pix_fmt", "yuv420p",
    "test_pipe_output.mp4"
]

print("Testing FFmpeg standalone pipeline...")
try:
    proc = subprocess.Popen(command, stdin=subprocess.PIPE)
    
    # Push 100 fake RGBA frames (blue screen) directly into stdin
    fake_frame = np.full((height, width, 4), [0, 0, 255, 255], dtype=np.uint8).tobytes()
    for _ in range(100):
        proc.stdin.write(fake_frame)
    
    proc.stdin.close()
    retcode = proc.wait()
    
    if retcode == 0:
        print("✅ SUCCESS: FFmpeg build, pipe handling, and encoder are working correctly!")
    else:
        print(f"❌ FAILED: FFmpeg exited with code {retcode}. Check encoder parameters/compilation.")

except Exception as e:
    print(f"❌ FAILED with Exception: {e}")