import subprocess
import glob
import os

FRAMES_DIR = "scene-1"
OUTPUT_FILE = "scene-1.mp4"
FPS = 30
FRAME_PATTERN = "frame%04d.png"  

frames = sorted(glob.glob(os.path.join(FRAMES_DIR, "*.png")))
if not frames:
    raise FileNotFoundError(f"No PNG frames found in {FRAMES_DIR}")

print(f"Found {len(frames)} frames")

subprocess.run([
    "ffmpeg",
    "-y",
    "-framerate", str(FPS),
    "-i", os.path.join(FRAMES_DIR, FRAME_PATTERN),
    "-c:v", "libx264",
    "-pix_fmt", "yuv420p",
    "-crf", "18",
    OUTPUT_FILE,
], check=True)

print(f"Written to {OUTPUT_FILE}")