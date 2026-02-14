"""
Quick viewer script to see the MuJoCo scene with the Franka Emika Panda robot.
Run from repo root: python backend/view_scene.py
Or from backend:  python view_scene.py

Use --camera both (default) to open two windows: scene view + robot hand view.
"""
import argparse
import os
import subprocess
import sys
import time

# Ensure backend dir is on path so "from sim import ..." works from any cwd
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("ERROR: MuJoCo not installed. Run: pip install mujoco>=3.1.3")
    sys.exit(1)

from sim import load_scene_with_table


def parse_args():
    parser = argparse.ArgumentParser(description="View MuJoCo Franka scene")
    parser.add_argument(
        "--camera",
        choices=["free", "hand", "both"],
        default="both",
        help="free=scene view, hand=robot hand camera, both=two windows (default)",
    )
    return parser.parse_args()


def run_viewer(model, data, camera_mode):
    """Run a single viewer with the given camera mode (free or hand)."""
    with mujoco.viewer.launch_passive(model, data) as viewer:
        hand_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "hand_camera")

        if camera_mode == "hand" and hand_cam_id >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = hand_cam_id
        else:
            # free / scene view
            viewer.cam.lookat[:] = [0.0, 0.0, 0.35]
            viewer.cam.distance = 2.2
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20

        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            dt = model.opt.timestep
            elapsed = time.time() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)


def main():
    args = parse_args()

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

    if args.camera == "both":
        print("\nLaunching two windows: scene view + robot hand view")
        print("Close either window to exit both.\n")
        script_path = os.path.abspath(__file__)
        child = subprocess.Popen(
            [sys.executable, script_path, "--camera", "hand"],
            cwd=os.path.dirname(script_path),
            creationflags=subprocess.CREATE_NEW_CONSOLE if sys.platform == "win32" else 0,
        )
        try:
            run_viewer(model, data, "free")
        finally:
            child.terminate()
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()
    else:
        title = "Robot hand view" if args.camera == "hand" else "Scene view"
        print(f"\nLaunching viewer: {title}")
        print("Controls: Mouse=rotate, Scroll=zoom, Right-drag=pan, Close window to exit\n")
        run_viewer(model, data, args.camera)

    print("\nViewer closed.")


if __name__ == "__main__":
    main()
