from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time
import numpy as np

from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.franka import Franka


# ------------------------------------------------------------
# 0) Rotation helper: roll (X), pitch (Y), yaw (Z) in DEGREES
# ------------------------------------------------------------

def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Convert roll/pitch/yaw (in degrees) about WORLD X/Y/Z into a quaternion [x, y, z, w].

    - roll  = rotation about X
    - pitch = rotation about Y
    - yaw   = rotation about Z

    Rotation order is Rz(yaw) * Ry(pitch) * Rx(roll) (standard yaw-pitch-roll).
    """
    r = np.deg2rad(roll_deg)
    p = np.deg2rad(pitch_deg)
    y = np.deg2rad(yaw_deg)

    cr = np.cos(r / 2.0)
    sr = np.sin(r / 2.0)
    cp = np.cos(p / 2.0)
    sp = np.sin(p / 2.0)
    cy = np.cos(y / 2.0)
    sy = np.sin(y / 2.0)

    # Quaternion math (ZYX order)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yq = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([x, yq, z, w], dtype=float)  # [x, y, z, w]


# ------------------------------------------------------------
# Global OFFSETS for BOTH Franka bases (translation)
# ------------------------------------------------------------

# WORLD coordinates (meters):
# +X: along table length
# +Y: across table width
# +Z: upward
FRANKA_MOVE_X = 0.0
FRANKA_MOVE_Y = 0.0
FRANKA_MOVE_Z = 0.0

# ------------------------------------------------------------
# Global ROTATIONS for BOTH Franka bases (this is what you wanted)
# ------------------------------------------------------------

# Rotations in DEGREES about WORLD axes:
FRANKA_ROT_X_DEG = 90.0   # roll  about world X
FRANKA_ROT_Y_DEG = 0.0   # pitch about world Y
FRANKA_ROT_Z_DEG = 0.0  # yaw   about world Z (spin around up axis)

# Example tweaks you can try later:
# FRANKA_ROT_Z_DEG = 90.0   # spin them 90° around Z
# FRANKA_ROT_Y_DEG = 10.0   # tilt them a bit around Y, etc.


# ------------------------------------------------------------
# 1) World and ground
# ------------------------------------------------------------

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()


# ------------------------------------------------------------
# 2) Table geometry (top + legs)
# ------------------------------------------------------------

# Table dimensions (meters)
TABLE_LENGTH = 1.2      # along X (long side)
TABLE_WIDTH = 1.0       # along Y (short side, made longer)
TABLE_THICKNESS = 0.05  # tabletop thickness (Z)
LEG_HEIGHT = 0.35       # leg height from floor
LEG_THICKNESS = 0.05    # square leg cross-section

# Table placement in world
TABLE_CENTER_X = 0.0
TABLE_CENTER_Y = 0.0

# Z positions
table_top_z = LEG_HEIGHT + TABLE_THICKNESS
table_center_z = LEG_HEIGHT + TABLE_THICKNESS / 2.0

# --- Tabletop ---
table = world.scene.add(
    FixedCuboid(
        prim_path="/World/TableTop",
        name="table_top",
        position=np.array([TABLE_CENTER_X, TABLE_CENTER_Y, table_center_z]),
        scale=np.array([TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS]),
        color=np.array([0.6, 0.4, 0.2]),
    )
)

# --- Table legs ---
leg_scale = np.array([LEG_THICKNESS, LEG_THICKNESS, LEG_HEIGHT])

half_len = TABLE_LENGTH / 2.0 - LEG_THICKNESS / 2.0
half_wid = TABLE_WIDTH / 2.0 - LEG_THICKNESS / 2.0

leg_xy_offsets = [
    (+half_len, +half_wid),
    (+half_len, -half_wid),
    (-half_len, +half_wid),
    (-half_len, -half_wid),
]

for i, (dx, dy) in enumerate(leg_xy_offsets, start=1):
    world.scene.add(
        FixedCuboid(
            prim_path=f"/World/TableLeg_{i}",
            name=f"table_leg_{i}",
            position=np.array([
                TABLE_CENTER_X + dx,
                TABLE_CENTER_Y + dy,
                LEG_HEIGHT / 2.0,
            ]),
            scale=leg_scale,
            color=np.array([0.4, 0.2, 0.1]),
        )
    )


# ------------------------------------------------------------
# 3) Two Franka arms on the -Y side (base pose + offsets + rotations)
# ------------------------------------------------------------

FRANKA_BASE_Z = table_top_z  # sit on top of the table

# Place them near the -Y edge, slightly inset so they’re on the tabletop
EDGE_INSET_Y = 0.15
BASE_Y = TABLE_CENTER_Y - (TABLE_WIDTH / 2.0 - EDGE_INSET_Y)  # -Y edge

# Along X (long side), symmetric around center
ARM_X_OFFSET = 0.3

# Base positions BEFORE offsets
left_franka_pos = np.array([
    TABLE_CENTER_X - ARM_X_OFFSET,
    BASE_Y,
    FRANKA_BASE_Z,
])

right_franka_pos = np.array([
    TABLE_CENTER_X + ARM_X_OFFSET,
    BASE_Y,
    FRANKA_BASE_Z,
])

# ---- APPLY MOVEMENT OFFSETS HERE ----
franka_offset = np.array([FRANKA_MOVE_X, FRANKA_MOVE_Y, FRANKA_MOVE_Z], dtype=float)
left_franka_pos += franka_offset
right_franka_pos += franka_offset

# ---- ORIENTATION FROM ROLL/PITCH/YAW (WORLD X/Y/Z) ----
franka_orientation = rpy_deg_to_quat(
    FRANKA_ROT_X_DEG,
    FRANKA_ROT_Y_DEG,
    FRANKA_ROT_Z_DEG,
)

left_franka = world.scene.add(
    Franka(
        prim_path="/World/FrankaLeft",
        name="franka_left",
        position=left_franka_pos,
        orientation=franka_orientation,
    )
)

right_franka = world.scene.add(
    Franka(
        prim_path="/World/FrankaRight",
        name="franka_right",
        position=right_franka_pos,
        orientation=franka_orientation,
    )
)


# ------------------------------------------------------------
# 4) Cube on the table (center)
# ------------------------------------------------------------

cube_size = 0.05
cube_center_z = table_top_z + cube_size / 2.0

world.scene.add(
    FixedCuboid(
        prim_path="/World/TestCube",
        name="test_cube",
        position=np.array([TABLE_CENTER_X, TABLE_CENTER_Y, cube_center_z]),
        scale=np.array([cube_size, cube_size, cube_size]),
        color=np.array([0.9, 0.2, 0.2]),
        # you can set "physics" type to dynamic if you want it to fall etc.
    )
)


# ------------------------------------------------------------
# 5) Run
# ------------------------------------------------------------

world.reset()
print(
    "Scene initialized:\n"
    "- Wider table (short side extended)\n"
    "- Two Franka arms on the -Y side, bases on the table\n"
    "- FRANKA_MOVE_X/Y/Z translate the bases in world coords\n"
    "- FRANKA_ROT_X/Y/Z (deg) rotate them about WORLD X/Y/Z"
)

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(1.0 / 60.0)

simulation_app.close()
