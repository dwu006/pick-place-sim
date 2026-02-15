"""
Interactive camera position recorder for MuJoCo scene.
Move the camera to your desired position, close the viewer, and it will print the settings.
"""
import mujoco
import mujoco.viewer
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim import load_scene_with_table

print("Loading scene...")
model, data = load_scene_with_table()

if model is None:
    print("ERROR: Could not load scene")
    sys.exit(1)

print("\n" + "="*60)
print("INTERACTIVE CAMERA RECORDER")
print("="*60)
print("\nControls:")
print("  - Left mouse drag: Rotate camera (azimuth/elevation)")
print("  - Right mouse drag: Pan camera (lookat point)")
print("  - Scroll wheel: Zoom in/out (distance)")
print("  - Double-click: Select point to look at")
print("  - ESC or close window: Save and quit")
print("\nMove the camera to your desired position, then close the window!")
print("="*60 + "\n")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set initial camera view (same as current default)
    viewer.cam.lookat[:] = [0.0, 0.0, 0.35]
    viewer.cam.distance = 2.2
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25

    print(f"Starting camera settings:")
    print(f"  lookat = [{viewer.cam.lookat[0]:.2f}, {viewer.cam.lookat[1]:.2f}, {viewer.cam.lookat[2]:.2f}]")
    print(f"  distance = {viewer.cam.distance:.2f}")
    print(f"  azimuth = {viewer.cam.azimuth:.1f}")
    print(f"  elevation = {viewer.cam.elevation:.1f}")
    print("\nAdjust the view, then close the window...\n")

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

    # Print final camera settings when viewer closes
    print("\n" + "="*60)
    print("FINAL CAMERA SETTINGS (copy these values):")
    print("="*60)
    print(f"lookat = [{viewer.cam.lookat[0]:.2f}, {viewer.cam.lookat[1]:.2f}, {viewer.cam.lookat[2]:.2f}]")
    print(f"distance = {viewer.cam.distance:.2f}")
    print(f"azimuth = {viewer.cam.azimuth:.1f}")
    print(f"elevation = {viewer.cam.elevation:.1f}")
    print("="*60 + "\n")

print("Done! Copy the values above and paste them here.")
