"""
Preview object_sim models (optional).

Run from repo root:
  python -m backend.sim.preview_object_sim              # list available objects
  python -m backend.sim.preview_object_sim cup         # open MuJoCo viewer for one object

For object_sim's full preview (all objects, camera rotation, click CLI), use the
preview that lives in object_sim/ — from repo root:
  python run_object_sim_preview.py -o cup    # one object
  python run_object_sim_preview.py           # cycle through all (requires click)

That uses from_xml_path() so mesh paths resolve relative to each object's folder.
Requires: object_sim cloned at repo root. pip install mujoco (and click for full preview).
"""
import os
import sys

# ensure backend on path so "sim" is importable (run from repo root: python -m backend.sim.preview_object_sim)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BACKEND_DIR = os.path.dirname(_SCRIPT_DIR)
if _BACKEND_DIR not in sys.path:
    sys.path.insert(0, _BACKEND_DIR)

from sim import get_object_sim_path, get_object_sim_object_path, OBJECT_SIM_OBJECT_NAMES


def main():
    obj_sim = get_object_sim_path()
    if not os.path.isdir(obj_sim):
        print(f"object_sim not found at {obj_sim}")
        print("Clone: git clone https://github.com/vikashplus/object_sim.git")
        return 1

    if len(sys.argv) > 1:
        name = sys.argv[1]
        path = get_object_sim_object_path(name)
        if not path:
            print(f"Object '{name}' not found. Try one of: {OBJECT_SIM_OBJECT_NAMES}")
            return 1
        try:
            import mujoco
            from mujoco import viewer
            model = mujoco.MjModel.from_xml_path(path)
            data = mujoco.MjData(model)
            print(f"Loaded {name} from object_sim. Close viewer to exit.")
            viewer.launch_passive(model, data)
        except ImportError:
            print("Install mujoco to preview: pip install mujoco")
            return 1
    else:
        available = [n for n in OBJECT_SIM_OBJECT_NAMES if get_object_sim_object_path(n)]
        print("object_sim available objects (use with: python -m backend.sim.preview_object_sim <name>):")
        for n in available:
            print(f"  {n}")
        if not available:
            print("  (none found)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
