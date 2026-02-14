"""
Preview object_sim objects with real meshes (cup, apple, etc.) — onscreen only.
Uses from_xml_path() so each object's XML is loaded from its folder and mesh paths resolve.
No skvideo or click required; only mujoco.

Usage (from repo root):
  python run_object_sim_preview.py cup       # preview one object (stays open until you close the window)
  python run_object_sim_preview.py          # list available objects

Requires: object_sim cloned at repo root, mujoco (pip install mujoco).
"""
import os
import sys
import time


def main():
    repo_root = os.path.dirname(os.path.abspath(__file__))
    object_sim_dir = os.path.join(repo_root, "object_sim")

    if not os.path.isdir(object_sim_dir):
        print("object_sim not found at", object_sim_dir)
        print("Clone: git clone https://github.com/vikashplus/object_sim.git")
        return 1

    try:
        import mujoco
        from mujoco import viewer
    except ImportError:
        print("MuJoCo not installed. Run: pip install mujoco")
        return 1

    # List object folders (object_sim subdirs that contain object.xml)
    def list_objects():
        names = []
        for name in sorted(os.listdir(object_sim_dir)):
            if name.startswith(".") or name in ("common.xml", "README.md", "preview.py", "videos"):
                continue
            path = os.path.join(object_sim_dir, name)
            if os.path.isdir(path) and os.path.isfile(os.path.join(path, "object.xml")):
                names.append(name)
        return names

    objects = list_objects()
    if not objects:
        print("No object_sim objects found (no object.xml in subdirs).")
        return 1

    # One object: -o cup or positional "cup"
    name = None
    for i, arg in enumerate(sys.argv[1:]):
        if arg in ("-o", "--object") and i + 2 < len(sys.argv):
            name = sys.argv[i + 2]
            break
        if not arg.startswith("-") and arg in objects:
            name = arg
            break

    if name:
        xml_path = os.path.join(object_sim_dir, name, "object.xml")
        if not os.path.isfile(xml_path):
            print(f"Object '{name}' not found. Available: {', '.join(objects[:12])}...")
            return 1
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        print(f"Loaded {name} from object_sim. Close the viewer window to exit.")
        with mujoco.viewer.launch_passive(model, data) as v:
            while v.is_running():
                mujoco.mj_step(model, data)
                v.sync()
                time.sleep(model.opt.timestep)
        return 0

    # No object: list available
    print("object_sim objects (run: python run_object_sim_preview.py <name>):")
    for n in objects:
        print(" ", n)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
