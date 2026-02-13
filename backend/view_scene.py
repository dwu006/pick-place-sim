"""
Quick viewer script to see the MuJoCo scene with the Franka Emika Panda robot.
Run: python view_scene.py
"""
import sys
import time

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("ERROR: MuJoCo not installed. Run: pip install mujoco>=3.1.3")
    sys.exit(1)

from sim import load_scene_with_table

print("Loading Franka Emika Panda scene...")
model, data = load_scene_with_table()

if model is None or data is None:
    print("\nERROR: Could not load scene.")
    print("\nOptions to fix this:")
    print("  1. Run setup script to download Franka FR3 files:")
    print("     python setup_mujoco.py")
    print("\n  2. Or clone the full mujoco_menagerie repo:")
    print("     git clone https://github.com/google-deepmind/mujoco_menagerie")
    print("\n  3. Or set MUJOCO_MENAGERIE_PATH env var to point to the repo")
    sys.exit(1)

print(f"OK Loaded model: {model.nbody} bodies, {model.ngeom} geoms")
print("\nLaunching viewer...")
print("Controls:")
print("  - Mouse: rotate camera")
print("  - Scroll: zoom")
print("  - Right-click + drag: pan")
print("  - Close window to exit\n")

# Launch viewer and run simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while viewer.is_running():
        step_start = time.time()

        # Step physics
        mujoco.mj_step(model, data)

        # Sync viewer
        viewer.sync()

        # Control frame rate (optional) using model timestep
        dt = model.opt.timestep
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\nViewer closed.")
