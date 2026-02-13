"""
MuJoCo simulation helper for Franka arm models from mujoco_menagerie.
Currently we use the Franka Emika Panda (`franka_emika_panda/scene.xml`),
which includes the arm and gripper.

Requires: clone/download `mujoco_menagerie` or run the setup script to fetch
the Panda files, then set MUJOCO_MENAGERIE_PATH to the repo root
(or leave unset to use ./mujoco_menagerie).
"""

import os

# Default: repo root / backend/sim -> repo root is parent of backend
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BACKEND_DIR = os.path.dirname(_SCRIPT_DIR)
_REPO_ROOT = os.path.dirname(_BACKEND_DIR)


def get_menagerie_path() -> str:
    return os.environ.get("MUJOCO_MENAGERIE_PATH", os.path.join(_REPO_ROOT, "mujoco_menagerie"))


def get_panda_scene_path() -> str:
    """Path to the Panda scene.xml in mujoco_menagerie."""
    return os.path.join(get_menagerie_path(), "franka_emika_panda", "scene.xml")


def load_scene_with_table():
    """
    Load the Franka Emika Panda scene from mujoco_menagerie.

    We load `franka_emika_panda/scene.xml`, then inject a simple box
    "table" under the robot so the arm appears mounted on a box.
    Returns (model, data) or (None, None) if the files are missing.
    """
    try:
        import mujoco
    except ImportError:
        return None, None

    menagerie = get_menagerie_path()
    panda_dir = os.path.join(menagerie, "franka_emika_panda")
    scene_path = os.path.join(panda_dir, "scene.xml")
    if not os.path.isfile(scene_path):
        return None, None

    # Read and augment scene XML
    with open(scene_path, "r") as f:
        xml = f.read()

    # Add a tall box under the base so the arm sits on it visually,
    # and a simple shelf behind the robot with a few "products".
    # Box: half-height = 0.35, top z = 0.3 => center z = 0.3 - 0.35 = -0.05
    shop_bodies = """
    <body name="shop_box" pos="0 0 -0.05">
      <geom name="shop_box_geom"
            type="box"
            size="0.75 0.75 0.35"
            rgba="0.55 0.45 0.35 1"/>
    </body>

    <!-- Simple shelf behind the robot: three levels, three columns -->
    <body name="shop_shelf" pos="0.8 0 0.02">
      <!-- Uprights (slightly smaller, still tall) -->
      <geom name="shelf_upright_left"  type="box" size="0.03 0.30 0.70" pos="0  0.30 0.70" rgba="0.5 0.5 0.5 1"/>
      <geom name="shelf_upright_right" type="box" size="0.03 0.30 0.70" pos="0 -0.30 0.70" rgba="0.5 0.5 0.5 1"/>
      <!-- Shelves (boards) - a bit smaller and lower -->
      <geom name="shelf_board_bottom" type="box" size="0.12 0.30 0.025" pos="-0.06 0.0 0.32" rgba="0.7 0.7 0.7 1"/>
      <geom name="shelf_board_mid"    type="box" size="0.12 0.30 0.025" pos="-0.06 0.0 0.74" rgba="0.7 0.7 0.7 1"/>
      <geom name="shelf_board_top"    type="box" size="0.12 0.30 0.025" pos="-0.06 0.0 1.16" rgba="0.7 0.7 0.7 1"/>
      <!-- Product cubes on the middle shelf, slightly lower -->
      <geom name="item_left"   type="box" size="0.04 0.04 0.07" pos="-0.10  0.22 0.83" rgba="0.9 0.4 0.4 1"/>
      <geom name="item_center" type="box" size="0.04 0.04 0.07" pos="-0.10  0.00 0.83" rgba="0.4 0.9 0.4 1"/>
      <geom name="item_right"  type="box" size="0.04 0.04 0.07" pos="-0.10 -0.22 0.83" rgba="0.4 0.4 0.9 1"/>
    </body>
"""
    if "</worldbody>" in xml:
        xml = xml.replace("</worldbody>", shop_bodies + "\n  </worldbody>")

    # Load from modified XML; includes and assets are resolved relative to panda_dir.
    cwd = os.getcwd()
    try:
        os.chdir(panda_dir)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        return model, data
    finally:
        os.chdir(cwd)
