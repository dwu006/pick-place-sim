"""
MuJoCo simulation helper for Franka arm models from mujoco_menagerie
and optional object models from object_sim.

- Franka: use `mujoco_menagerie` (MUJOCO_MENAGERIE_PATH or ./mujoco_menagerie).
- Objects: clone [object_sim](https://github.com/vikashplus/object_sim) into
  the repo root; set OBJECT_SIM_PATH or use ./object_sim.

- Main scene (load_scene_with_table): uses primitives (box/cylinder/sphere) so
  no mesh paths; object_sim STL meshes are not loaded there.
- To preview object_sim meshes (cup, apple, etc.): run object_sim's preview —
  from repo root: python run_object_sim_preview.py -o cup
  That uses object_sim/preview.py, which loads each object via from_xml_path()
  so mesh paths resolve relative to that object's folder.
"""

import os
import random
import struct
from typing import Optional

# Default: repo root / backend/sim -> repo root is parent of backend
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BACKEND_DIR = os.path.dirname(_SCRIPT_DIR)
_REPO_ROOT = os.path.dirname(_BACKEND_DIR)


def get_menagerie_path() -> str:
    return os.environ.get("MUJOCO_MENAGERIE_PATH", os.path.join(_REPO_ROOT, "mujoco_menagerie"))


def get_object_sim_path() -> str:
    """Path to object_sim repo (MuJoCo models for daily objects). Clone from https://github.com/vikashplus/object_sim"""
    return os.environ.get("OBJECT_SIM_PATH", os.path.join(_REPO_ROOT, "object_sim"))


def get_object_sim_object_path(object_name: str) -> Optional[str]:
    """Path to object_sim/{object_name}/object.xml, or None if not present."""
    base = get_object_sim_path()
    path = os.path.join(base, object_name, "object.xml")
    return path if os.path.isfile(path) else None


# object_sim object names we can use for cleanup room (subset of object_sim repo)
OBJECT_SIM_OBJECT_NAMES = [
    "banana", "duck", "phone", "elephant", "eyeglasses", "flute",
    "gamecontroller", "headphones", "mouse", "piggybank", "pyramidlarge",
    "stanfordbunny", "train", "watch", "airplane", "alarmclock", "camera",
]


def get_panda_scene_path() -> str:
    """Path to the Panda scene.xml in mujoco_menagerie."""
    return os.path.join(get_menagerie_path(), "franka_emika_panda", "scene.xml")


# Box: pos 0 0 -0.05, size (half-extents) 0.9 0.9 0.35 → x,y in [-0.9, 0.9], top z=0.30.
# Bin on box at (0.5, -0.35). Robot base at origin. Objects placed randomly on box top.
# object_sim: (object_name, mesh_file, rgba) — positions assigned randomly on box in _object_sim_assets_and_bodies_xml.
_OBJECT_SIM_TABLE_ITEMS = [
    ("banana", "banana.stl", "0.95 0.88 0.25 1"),
    ("duck", "duck.stl", "0.98 0.75 0.2 1"),
    ("phone", "phone.stl", "0.18 0.18 0.22 1"),
    ("elephant", "elephant.stl", "0.5 0.5 0.55 1"),
    ("eyeglasses", "eyeglasses.stl", "0.25 0.25 0.3 1"),
    ("flute", "flute.stl", "0.75 0.65 0.45 1"),
    ("gamecontroller", "gamecontroller.stl", "0.2 0.2 0.25 1"),
    ("headphones", "headphones.stl", "0.22 0.22 0.26 1"),
    ("mouse", "mouse.stl", "0.35 0.35 0.4 1"),
    ("piggybank", "piggybank.stl", "0.95 0.5 0.55 1"),
    ("pyramidlarge", "pyramidlarge.stl", "0.85 0.6 0.2 1"),
    ("stanfordbunny", "stanfordbunny.stl", "0.95 0.92 0.88 1"),
    ("train", "train.stl", "0.25 0.41 0.87 1"),
    ("watch", "watch.stl", "0.87 0.85 0.8 1"),
    ("airplane", "airplane.stl", "0.7 0.75 0.8 1"),
    ("alarmclock", "alarmclock.stl", "0.9 0.25 0.2 1"),
    ("camera", "camera.stl", "0.2 0.2 0.22 1"),
]

# Box platform: Objects must be within robot reach (~0.7m from origin).
# Box top surface is z=0.30; each object's z is computed so mesh bottom sits on the surface.
_BOX_HALF = 0.9
_BOX_TOP_SURFACE = 0.30
_ROBOT_BASE_MARGIN = 0.35   # keep objects outside |x|<0.35, |y|<0.35 (robot base)
_ROBOT_MAX_REACH = 0.65     # max distance from origin for reliable picking
_BIN_X_LO, _BIN_X_HI = 0.32, 0.68   # bin roughly at 0.5 ± 0.18
_BIN_Y_LO, _BIN_Y_HI = -0.52, -0.18  # bin roughly at -0.35 ± 0.17
_MIN_OBJECT_SEP = 0.12      # min distance between object centers


def _stl_mesh_z_min(filepath: str) -> float:
    """Return the minimum z coordinate of all vertices in an STL file (mesh-local frame).
    Used to place object bodies so the mesh bottom sits on the box surface."""
    z_vals: list[float] = []
    try:
        with open(filepath, "rb") as f:
            raw = f.read()
        # Try ASCII: lines like "  vertex x y z"
        for line in raw.split(b"\n"):
            line = line.decode("utf-8", errors="ignore").strip()
            if line.startswith("vertex "):
                parts = line.split()
                if len(parts) >= 4:
                    z_vals.append(float(parts[3]))
        # If no vertices found, try binary STL (80 header + 4-byte n + 50 bytes per triangle)
        if not z_vals and len(raw) >= 84:
            n = struct.unpack("<I", raw[80:84])[0]
            if 0 < n <= (len(raw) - 84) // 50:
                for i in range(n):
                    off = 84 + i * 50  # normal 12, then v1 v2 v3 (12 bytes each)
                    for v in range(3):
                        z_vals.append(struct.unpack("<fff", raw[off + 12 + v * 12 : off + 12 + (v + 1) * 12])[2])
    except Exception:
        pass
    return min(z_vals) if z_vals else 0.0


def _random_positions_on_box(n: int, seed: Optional[int] = 42) -> list[tuple[float, float]]:
    """Return n random (x, y) positions on the box top, within robot reach, avoiding robot base and bin."""
    rng = random.Random(seed)
    out: list[tuple[float, float]] = []
    max_attempts = n * 200
    attempts = 0
    while len(out) < n and attempts < max_attempts:
        attempts += 1
        x = rng.uniform(-_ROBOT_MAX_REACH, _ROBOT_MAX_REACH)
        y = rng.uniform(-_ROBOT_MAX_REACH, _ROBOT_MAX_REACH)
        dist = (x**2 + y**2) ** 0.5
        # Must be within reach but outside robot base
        if dist < _ROBOT_BASE_MARGIN or dist > _ROBOT_MAX_REACH:
            continue
        # Avoid bin area
        if _BIN_X_LO <= x <= _BIN_X_HI and _BIN_Y_LO <= y <= _BIN_Y_HI:
            continue
        # Check separation from other objects
        too_close = False
        for (ox, oy) in out:
            if (x - ox) ** 2 + (y - oy) ** 2 < _MIN_OBJECT_SEP ** 2:
                too_close = True
                break
        if too_close:
            continue
        out.append((x, y))

    # Fallback: place remaining objects in a ring pattern
    import math
    while len(out) < n:
        i = len(out)
        angle = (i * 2.0 * math.pi / max(n, 1)) + 0.3
        # Alternate between inner and outer ring
        r = _ROBOT_BASE_MARGIN + 0.15 + (i % 3) * 0.12
        r = min(r, _ROBOT_MAX_REACH - 0.05)
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        # Avoid bin area
        if _BIN_X_LO <= x <= _BIN_X_HI and _BIN_Y_LO <= y <= _BIN_Y_HI:
            y = _BIN_Y_HI + 0.08
        too_close = any(
            (x - ox) ** 2 + (y - oy) ** 2 < _MIN_OBJECT_SEP ** 2 for (ox, oy) in out
        )
        if not too_close:
            out.append((x, y))
        else:
            # Shift slightly
            x += 0.1 * math.cos(angle + 0.5)
            y += 0.1 * math.sin(angle + 0.5)
            out.append((x, y))
    return out[:n]
# Primitives fallback (name, geom type, size args, pos x y z, rgba)
_ROOM_TABLE_OBJECTS = [
    ("obj_cup", "cylinder", "0.025 0.04", "1.05 0.22 0.36", "0.85 0.75 0.7 1"),
    ("obj_mug", "cylinder", "0.022 0.035", "1.20 0.22 0.355", "0.6 0.5 0.45 1"),
    ("obj_bottle", "cylinder", "0.015 0.05", "1.35 0.22 0.36", "0.4 0.6 0.9 1"),
    ("obj_waterbottle", "cylinder", "0.018 0.045", "1.05 0.38 0.36", "0.5 0.75 0.95 1"),
    ("obj_bowl", "cylinder", "0.04 0.02", "1.20 0.38 0.35", "0.9 0.85 0.8 1"),
    ("obj_teapot", "box", "0.03 0.025 0.03", "1.35 0.38 0.36", "0.7 0.5 0.4 1"),
    ("obj_cube_red", "box", "0.03 0.03 0.03", "1.05 0.54 0.35", "0.9 0.25 0.2 1"),
    ("obj_cube_blue", "box", "0.025 0.025 0.035", "1.20 0.54 0.355", "0.2 0.4 0.9 1"),
    ("obj_cube_green", "box", "0.02 0.02 0.025", "1.35 0.54 0.35", "0.25 0.7 0.4 1"),
    ("obj_apple", "sphere", "0.028", "1.10 0.40 0.358", "0.85 0.2 0.15 1"),
    ("obj_banana", "box", "0.04 0.015 0.015", "1.25 0.40 0.35", "0.95 0.85 0.2 1"),
    ("obj_duck", "sphere", "0.022", "1.40 0.40 0.352", "0.95 0.75 0.2 1"),
    ("obj_stapler", "box", "0.035 0.02 0.015", "1.15 0.28 0.355", "0.5 0.5 0.55 1"),
    ("obj_phone", "box", "0.025 0.04 0.008", "1.30 0.28 0.354", "0.2 0.2 0.25 1"),
    ("obj_toothpaste", "cylinder", "0.012 0.04", "1.08 0.46 0.36", "0.95 0.95 1 1"),
    ("obj_clock", "cylinder", "0.025 0.01", "1.22 0.46 0.35", "0.9 0.88 0.85 1"),
    ("obj_scissors", "box", "0.03 0.02 0.008", "1.38 0.46 0.354", "0.6 0.6 0.65 1"),
]


def _object_sim_assets_and_bodies_xml(panda_dir: str):
    """Build (assets_xml, bodies_xml) for object_sim meshes. Use absolute paths so MuJoCo finds STLs when loading the composite.
    Objects are placed randomly on the box platform, avoiding the robot base and bin."""
    base = get_object_sim_path()
    if not os.path.isdir(base):
        return "", ""
    items_with_mesh = [
        (obj_name, mesh_file, rgba)
        for obj_name, mesh_file, rgba in _OBJECT_SIM_TABLE_ITEMS
        if os.path.isfile(os.path.join(base, obj_name, mesh_file))
    ]
    if not items_with_mesh:
        return "", ""
    xy_positions = _random_positions_on_box(len(items_with_mesh))
    asset_parts = []
    body_parts = []
    for (obj_name, mesh_file, rgba), (x, y) in zip(items_with_mesh, xy_positions):
        mesh_path = os.path.join(base, obj_name, mesh_file)
        file_path = os.path.abspath(mesh_path).replace("\\", "/")
        # Place body so mesh bottom sits on box surface (z=0.30): body_z = box_top - mesh_z_min
        z_min = _stl_mesh_z_min(mesh_path)
        z = _BOX_TOP_SURFACE - z_min
        mesh_name = f"room_obj_{obj_name}"
        body_name = f"obj_{obj_name}"
        pos_str = f"{x:.3f} {y:.3f} {z:.3f}"
        asset_parts.append(f'    <mesh name="{mesh_name}" file="{file_path}"/>')
        body_parts.append(
            f'    <body name="{body_name}" pos="{pos_str}">\n'
            f'      <geom name="{body_name}_g" type="mesh" mesh="{mesh_name}" rgba="{rgba}"/>\n'
            f"    </body>"
        )
    assets = "\n".join(asset_parts) if asset_parts else ""
    bodies = "\n".join(body_parts) if body_parts else ""
    return assets, bodies


def _room_table_objects_xml() -> str:
    """Primitive geoms for cleanup objects on the table (no STL; distinct shapes and colors)."""
    parts = []
    for name, geom_type, size, pos, rgba in _ROOM_TABLE_OBJECTS:
        if geom_type == "sphere":
            geom_tag = f'<geom name="{name}_g" type="sphere" size="{size}" rgba="{rgba}"/>'
        elif geom_type == "cylinder":
            geom_tag = f'<geom name="{name}_g" type="cylinder" size="{size}" rgba="{rgba}"/>'
        else:
            geom_tag = f'<geom name="{name}_g" type="box" size="{size}" rgba="{rgba}"/>'
        parts.append(f'    <body name="{name}" pos="{pos}">\n      {geom_tag}\n    </body>')
    return "\n".join(parts)


# Opening tag of the hand body in panda.xml (insert wrist camera + visible marker as first children)
# Hand quat (w,x,y,z)=(.9239,0,0,-.3827) ~ 45 deg around Z; so hand -Z is world down (0,0,-1).
_HAND_BODY_OPEN = '<body name="hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">'
# Camera on wrist: same quat as ray (0 1 0 0 = 180 deg X) so it looks down at gripper; ray points down.
_WRIST_CAMERA_INSERT = (
    '<body name="hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">\n'
    '      <camera name="hand_camera" pos="0.04 0 -0.01" quat="0 1 0 0" fovy="60" resolution="640 480"/>\n'
    '      <body name="hand_camera_marker" pos="0.04 0 -0.01">\n'
    '        <geom name="hand_camera_vis" type="sphere" size="0.018" rgba="1 0 0 0.85" contype="0" conaffinity="0" group="2"/>\n'
    '        <body name="hand_camera_ray" pos="0 0 0" quat="0 1 0 0">\n'
    '          <geom name="hand_camera_look" type="cylinder" size="0.004 0.08" pos="0 0 -0.08" rgba="0 0.9 0 0.9" contype="0" conaffinity="0" group="2"/>\n'
    '        </body>\n'
    '      </body>'
)


def _ensure_panda_wrist_cam_xml(panda_dir: str) -> bool:
    """Generate _panda_wrist_cam.xml with camera inside hand body. Returns True if successful."""
    panda_path = os.path.join(panda_dir, "panda.xml")
    out_path = os.path.join(panda_dir, "_panda_wrist_cam.xml")
    if not os.path.isfile(panda_path):
        return False
    try:
        with open(panda_path, "r") as f:
            content = f.read()
        if _HAND_BODY_OPEN not in content:
            return False
        content = content.replace(_HAND_BODY_OPEN, _WRIST_CAMERA_INSERT, 1)
        with open(out_path, "w") as f:
            f.write(content)
        return True
    except Exception:
        return False


def load_scene_with_table():
    """
    Load the Franka Emika Panda scene from mujoco_menagerie with a cleanup room.

    We load `franka_emika_panda/scene.xml`, then inject: a box as floor (robot base),
    a table with primitive cleanup objects (box/cylinder/sphere, distinct colors) off the box at x=1.2, and a bin on the box.
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

    # Read Panda scene
    with open(scene_path, "r") as f:
        xml_base = f.read()

    # Use panda with wrist camera (camera inside hand body) if generation succeeds
    if _ensure_panda_wrist_cam_xml(panda_dir):
        xml_base = xml_base.replace('file="panda.xml"', 'file="_panda_wrist_cam.xml"')

    # Room structure: box, table, objects (object_sim meshes or primitives), bin
    obj_assets_xml, obj_bodies_xml = _object_sim_assets_and_bodies_xml(panda_dir)
    if obj_assets_xml and obj_bodies_xml:
        # Use object_sim meshes: write composite to file so mesh paths resolve, then load from file
        room_bodies_obj = """
    <!-- Box as floor (robot base), x,y in [-0.9, 0.9], top at z = 0.30 -->
    <body name="shop_box" pos="0 0 -0.05">
      <geom name="shop_box_geom" type="box" size="0.9 0.9 0.35" rgba="0.55 0.45 0.35 1"/>
    </body>
""" + obj_bodies_xml + """
    <!-- Bin on box (base at 0.30) -->
    <body name="room_bin" pos="0.5 -0.35 0.32">
      <geom name="bin_base" type="box" size="0.12 0.12 0.02" rgba="0.35 0.35 0.38 1"/>
      <geom name="bin_wall_n" type="box" size="0.12 0.01 0.1" pos="0 0.11 0.06" rgba="0.4 0.4 0.42 1"/>
      <geom name="bin_wall_s" type="box" size="0.12 0.01 0.1" pos="0 -0.11 0.06" rgba="0.4 0.4 0.42 1"/>
      <geom name="bin_wall_e" type="box" size="0.01 0.12 0.1" pos="0.11 0 0.06" rgba="0.4 0.4 0.42 1"/>
      <geom name="bin_wall_w" type="box" size="0.01 0.12 0.1" pos="-0.11 0 0.06" rgba="0.4 0.4 0.42 1"/>
    </body>
"""
        xml = xml_base
        if "</asset>" in xml:
            xml = xml.replace("</asset>", obj_assets_xml + "\n  </asset>")
        else:
            xml = xml.replace("<worldbody>", "<asset>\n" + obj_assets_xml + "\n  </asset>\n  <worldbody>")
        xml = xml.replace("</worldbody>", room_bodies_obj + "\n  </worldbody>")
        composite_path = os.path.join(panda_dir, "_cleanup_room_objects.xml")
        with open(composite_path, "w") as f:
            f.write(xml)
        try:
            model = mujoco.MjModel.from_xml_path(composite_path)
            data = mujoco.MjData(model)
            return model, data
        except Exception:
            os.remove(composite_path)
            raise
        # (cleanup temp file on success optional; leave for debugging)

    # Fallback: primitives (no object_sim or no meshes found)
    room_bodies = """
    <!-- Box as floor (robot base), x,y in [-0.9, 0.9], top at z = 0.30 -->
    <body name="shop_box" pos="0 0 -0.05">
      <geom name="shop_box_geom" type="box" size="0.9 0.9 0.35" rgba="0.55 0.45 0.35 1"/>
    </body>
""" + _room_table_objects_xml() + """
    <!-- Bin on box (base at 0.30) -->
    <body name="room_bin" pos="0.5 -0.35 0.32">
      <geom name="bin_base" type="box" size="0.12 0.12 0.02" rgba="0.35 0.35 0.38 1"/>
      <geom name="bin_wall_n" type="box" size="0.12 0.01 0.1" pos="0 0.11 0.06" rgba="0.4 0.4 0.42 1"/>
      <geom name="bin_wall_s" type="box" size="0.12 0.01 0.1" pos="0 -0.11 0.06" rgba="0.4 0.4 0.42 1"/>
      <geom name="bin_wall_e" type="box" size="0.01 0.12 0.1" pos="0.11 0 0.06" rgba="0.4 0.4 0.42 1"/>
      <geom name="bin_wall_w" type="box" size="0.01 0.12 0.1" pos="-0.11 0 0.06" rgba="0.4 0.4 0.42 1"/>
    </body>
"""
    xml = xml_base.replace("</worldbody>", room_bodies + "\n  </worldbody>")
    cwd = os.getcwd()
    try:
        os.chdir(panda_dir)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        return model, data
    except Exception:
        raise
    finally:
        os.chdir(cwd)
