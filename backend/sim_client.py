"""
Real-time MuJoCo simulation client that connects to Vultr backend via WebSocket.
Receives robot steps and moves the Franka arm in real-time.

Object positions: Set when the scene loads (backend/sim uses random layout with seed 42,
so same layout every run). The client reads actual positions from the model at startup.
Use controller.reset_objects() to restore objects to initial positions.

Usage:
  python sim_client.py <order_id> [--backend-url ws://YOUR_VULTR_IP:8000]

Example:
  python sim_client.py abc123 --backend-url ws://149.28.xx.xx:8000
"""
import argparse
import asyncio
import io
import json
import os
import sys
import time
import threading
from typing import Optional

try:
    import httpx
    from PIL import Image
except ImportError:
    httpx = None
    Image = None

# Ensure backend dir is on path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:
    import mujoco
    import mujoco.viewer
    import websockets
    import numpy as np
except ImportError as e:
    print(f"ERROR: Missing dependency: {e}")
    print("Run: pip install mujoco websockets")
    sys.exit(1)

from sim import load_scene_with_table, OBJECT_SIM_OBJECT_NAMES

# Default object positions - ADJUSTED to be within Franka Panda reach (~0.75m from base)
# Robot base is at origin (0,0,0.3). Max comfortable reach is ~0.7m.
# Objects placed in a ring around the robot, avoiding base area and bin.
DEFAULT_OBJECT_POSITIONS = {
    # Front area (y negative, x near zero)
    "banana": [0.15, -0.55, 0.32],
    "duck": [0.35, -0.50, 0.32],
    "mouse": [-0.15, -0.55, 0.32],

    # Front-right area
    "phone": [0.50, -0.35, 0.32],
    "alarmclock": [0.55, -0.15, 0.32],

    # Right area
    "headphones": [0.55, 0.15, 0.32],
    "eyeglasses": [0.50, 0.35, 0.32],

    # Front-left area
    "watch": [-0.50, -0.35, 0.32],
    "flute": [-0.55, -0.15, 0.32],

    # Left area
    "gamecontroller": [-0.55, 0.15, 0.32],
    "piggybank": [-0.50, 0.35, 0.32],

    # Back area (y positive)
    "elephant": [-0.35, 0.50, 0.32],
    "train": [-0.15, 0.55, 0.32],
    "stanfordbunny": [0.15, 0.55, 0.32],
    "camera": [0.35, 0.50, 0.32],

    # Near robot but reachable
    "airplane": [0.40, 0.20, 0.32],
    "pyramidlarge": [-0.40, 0.20, 0.32],
}

# Runtime positions (can be modified as objects are moved)
OBJECT_POSITIONS = {}

# Bin position
BIN_POS = [0.5, -0.35, 0.40]  # slightly above the bin

# Home position for the arm (joint angles in radians)
HOME_QPOS = [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04]

# Gripper open/close values
GRIPPER_OPEN = 0.04
GRIPPER_CLOSE = 0.005

# Franka Panda joint limits: J1[-2.9,2.9], J2[-1.76,1.76], J3[-2.9,2.9], J4[-3.07,-0.07], J5[-2.9,2.9], J6[-0.02,3.75], J7[-2.9,2.9]
# Robot base is elevated at z=0.3, objects are at z~0.32 (box surface at z=0.30)

# TESTED joint configurations for Franka Panda (VERIFIED via FK)
# Format: [J1, J2, J3, J4, J5, J6, J7]
# J1 = base rotation, J2 = shoulder, J3 = upper arm rotation, J4 = elbow, J5 = forearm rotation, J6 = wrist, J7 = flange
# Robot base at z=0.3, objects at z=0.32

# Home: arm folded up safely (z=0.92)
HOME_JOINTS = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785]

# Reaching down pose: arm bending DOWN to table level
# J2=1.3 (shoulder forward), J4=-1.3 (elbow extended), J6=3.5 (wrist down)
# Result: z=0.35, reach=0.73m (perfect for picking at table level z=0.32)
REACH_DOWN_JOINTS = [0.0, 1.3, 0.0, -1.3, 0.0, 3.5, 0.785]

# Lifted pose: holding object up high (z=0.92)
# J2=-0.2 (shoulder back), J4=-1.8 (elbow bent), J6=1.5 (wrist level)
LIFTED_JOINTS = [0.0, -0.2, 0.0, -1.8, 0.0, 1.5, 0.785]

# Over bin pose: rotated to bin and lowered (z=0.52)
# J1=-0.6 (rotate to bin), J2=1.0, J4=-1.3, J6=3.0
BIN_JOINTS = [-0.6, 1.0, 0.0, -1.3, 0.0, 3.0, 0.785]

PICK_CONFIGS = {
    "home": HOME_JOINTS + [GRIPPER_OPEN, GRIPPER_OPEN],
    "reach_down": REACH_DOWN_JOINTS + [GRIPPER_OPEN, GRIPPER_OPEN],
    "lifted": LIFTED_JOINTS + [GRIPPER_CLOSE, GRIPPER_CLOSE],
    "bin": BIN_JOINTS + [GRIPPER_CLOSE, GRIPPER_CLOSE],
}


class RobotController:
    """Controls the Franka Panda robot arm in MuJoCo."""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.current_step = None
        self.target_qpos = None
        self.gripper_target = GRIPPER_OPEN
        self.step_queue = []
        self.lock = threading.Lock()
        self.held_object = None  # Currently held object body id
        self.object_body_ids = {}  # name -> body id
        self.initial_object_body_pos = {}  # body_id -> (x,y,z) for reset

        # Step simulation once to compute positions
        mujoco.mj_forward(model, data)

        # Find object body positions and store initial positions for reset
        self._find_object_positions()

        # Initialize arm to home position
        self._set_arm_qpos(HOME_QPOS)
        mujoco.mj_forward(model, data)

    def _find_object_positions(self):
        """Find positions of all objects in the scene using defaults."""
        # Start with default positions
        for obj_name, pos in DEFAULT_OBJECT_POSITIONS.items():
            OBJECT_POSITIONS[obj_name] = list(pos)

        # Find body IDs and update positions if available
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and name.startswith("obj_"):
                obj_name = name[4:]  # remove "obj_" prefix
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                self.object_body_ids[obj_name] = body_id

                # Try to get position from simulation
                pos = self.data.xpos[body_id].copy()
                if pos[0] != 0 or pos[1] != 0 or pos[2] != 0:
                    OBJECT_POSITIONS[obj_name] = list(pos)
                # Store initial position for reset (model.body_pos is the canonical source)
                self.initial_object_body_pos[body_id] = self.model.body_pos[body_id].copy()

        # Print all object positions
        print(f"  Loaded {len(OBJECT_POSITIONS)} objects:")
        for obj_name, pos in OBJECT_POSITIONS.items():
            print(f"    {obj_name}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

    def _set_arm_qpos(self, qpos):
        """Set arm joint positions directly for accurate IK positioning."""
        # Set joint positions directly (not via actuators) for precise control
        for i in range(min(9, len(qpos))):
            self.data.qpos[i] = qpos[i]
        mujoco.mj_forward(self.model, self.data)

    def _get_arm_qpos(self):
        """Get current arm joint positions."""
        return self.data.qpos[:9].copy()

    def _interpolate_to_target(self, target_qpos, steps=50):
        """Smoothly interpolate from current position to target."""
        current = self._get_arm_qpos()
        for i in range(steps):
            t = (i + 1) / steps
            # Linear interpolation
            interp = current + t * (target_qpos - current)
            self._set_arm_qpos(interp)
            yield

    def _solve_ik(self, target_pos, max_iterations=100, tolerance=0.01):
        """
        Solve inverse kinematics using Jacobian pseudo-inverse method.
        Returns joint angles that place the end effector at target_pos.
        """
        import math

        # Get end effector body ID
        ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        if ee_body_id < 0:
            ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link7")

        # Joint limits for Franka Panda
        joint_limits = [
            (-2.8973, 2.8973),   # J1
            (-1.7628, 1.7628),   # J2
            (-2.8973, 2.8973),   # J3
            (-3.0718, -0.0698),  # J4
            (-2.8973, 2.8973),   # J5
            (-0.0175, 3.7525),   # J6
            (-2.8973, 2.8973),   # J7
        ]

        # Start from a good initial guess: rotate to face target, use reaching pose
        angle = math.atan2(target_pos[1], target_pos[0])
        init_qpos = np.array([angle, 0.8, 0.0, -1.5, 0.0, 2.5, 0.785])

        # Save current state
        saved_qpos = self.data.qpos.copy()

        # Set initial guess
        for i in range(7):
            self.data.qpos[i] = init_qpos[i]
        mujoco.mj_forward(self.model, self.data)

        # Iterative IK using Jacobian
        step_size = 0.5
        for iteration in range(max_iterations):
            # Get current end effector position
            ee_pos = self.data.xpos[ee_body_id].copy()

            # Offset for gripper (target is where object is, ee is hand body)
            ee_pos[2] -= 0.08  # Account for gripper length

            # Compute error
            error = target_pos - ee_pos
            error_norm = np.linalg.norm(error)

            if error_norm < tolerance:
                # Success!
                result = self.data.qpos[:7].copy()
                # Restore state
                self.data.qpos[:] = saved_qpos
                mujoco.mj_forward(self.model, self.data)
                return result

            # Compute Jacobian for end effector position
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, ee_body_id)

            # Extract just the arm joints (first 7 columns)
            J = jacp[:, :7]

            # Compute pseudo-inverse
            J_pinv = np.linalg.pinv(J)

            # Compute joint velocity
            dq = J_pinv @ error * step_size

            # Update joints with clamping
            for i in range(7):
                new_q = self.data.qpos[i] + dq[i]
                self.data.qpos[i] = max(joint_limits[i][0], min(joint_limits[i][1], new_q))

            mujoco.mj_forward(self.model, self.data)

        # If we didn't converge, still return the best we found
        result = self.data.qpos[:7].copy()
        # Restore state
        self.data.qpos[:] = saved_qpos
        mujoco.mj_forward(self.model, self.data)

        final_ee = self.data.xpos[ee_body_id].copy()
        print(f"    IK warning: converged to {np.linalg.norm(target_pos - final_ee):.3f}m error")
        return result

    def update_held_object(self):
        """Held object follows gripper exactly."""
        if self.held_object and self.held_object in self.object_body_ids:
            body_id = self.object_body_ids[self.held_object]
            ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand")
            if ee_body_id >= 0:
                ee_pos = self.data.xpos[ee_body_id].copy()
                ee_pos[2] -= 0.05
                self.model.body_pos[body_id] = ee_pos
                mujoco.mj_forward(self.model, self.data)

    def reset_objects(self):
        """Reset all objects to their initial positions (as loaded from the scene)."""
        for body_id, pos in self.initial_object_body_pos.items():
            self.model.body_pos[body_id][:] = pos
        mujoco.mj_forward(self.model, self.data)
        # Refresh OBJECT_POSITIONS so IK targets are correct
        for obj_name, body_id in self.object_body_ids.items():
            OBJECT_POSITIONS[obj_name] = list(self.data.xpos[body_id])
        self.held_object = None
        print("  Objects reset to initial positions.")

    def add_step(self, step_type: str, item_id: str, message: str):
        """Add a robot step to the queue."""
        with self.lock:
            self.step_queue.append({
                "step": step_type,
                "item_id": item_id,
                "message": message
            })

    def process_step(self):
        """Process the next step in the queue. Returns generator for animation frames."""
        import math

        with self.lock:
            if not self.step_queue:
                return None
            step = self.step_queue.pop(0)

        step_type = step["step"]
        item_id = step["item_id"]

        print(f"  Executing: {step_type} - {item_id}")

        if step_type == "move_to_pick":
            # Use IK to compute exact joint angles to reach the object
            if item_id in OBJECT_POSITIONS:
                target_pos = np.array(OBJECT_POSITIONS[item_id])
                print(f"    Moving to {item_id} at [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")

                self.gripper_target = GRIPPER_OPEN

                # Go directly to the object
                grasp_target = target_pos.copy()

                # Calculate rotation angle to face object
                angle = math.atan2(target_pos[1], target_pos[0])

                # Phase 1: Rotate base to face object (keep arm up)
                rotate_qpos = np.array(HOME_JOINTS + [GRIPPER_OPEN, GRIPPER_OPEN])
                rotate_qpos[0] = angle

                # Phase 2: Solve IK for exact object position
                ik_joints = self._solve_ik(grasp_target)
                reach_qpos = np.zeros(9)
                reach_qpos[:7] = ik_joints
                reach_qpos[7] = GRIPPER_OPEN
                reach_qpos[8] = GRIPPER_OPEN

                print(f"    IK solved: J1={ik_joints[0]:.2f} J2={ik_joints[1]:.2f} J4={ik_joints[3]:.2f} J6={ik_joints[5]:.2f}")

                def move_to_object():
                    # Rotate to face object
                    for _ in self._interpolate_to_target(rotate_qpos, steps=30):
                        yield
                    # Reach down to object
                    for _ in self._interpolate_to_target(reach_qpos, steps=40):
                        yield

                return move_to_object()
            else:
                print(f"    WARNING: Object {item_id} not found in positions!")
                return None

        elif step_type == "pick":
            # Close gripper, then attach object (object attaches AFTER gripper closes)
            self.gripper_target = GRIPPER_CLOSE
            print(f"    Closing gripper on {item_id}")

            current = self._get_arm_qpos()
            target = current.copy()
            target[7] = GRIPPER_CLOSE
            target[8] = GRIPPER_CLOSE

            def close_and_grab():
                # First close the gripper
                for _ in self._interpolate_to_target(target, steps=20):
                    yield
                # THEN attach the object (after gripper is closed)
                if item_id in self.object_body_ids:
                    self.held_object = item_id
                    print(f"    Grabbed {item_id}!")

            return close_and_grab()

        elif step_type == "move_to_delivery":
            # SEQUENCE: Lift up → Move to bin position using IK
            self.gripper_target = GRIPPER_CLOSE

            current = self._get_arm_qpos()
            current_angle = current[0]

            # Phase 1: Lift object up (keep same rotation)
            lift_qpos = np.array(LIFTED_JOINTS + [GRIPPER_CLOSE, GRIPPER_CLOSE])
            lift_qpos[0] = current_angle

            # Phase 2: Use IK to reach bin position
            bin_target = np.array(BIN_POS)
            bin_target[2] += 0.15  # 15cm above bin for safe drop
            bin_ik_joints = self._solve_ik(bin_target)
            bin_qpos = np.zeros(9)
            bin_qpos[:7] = bin_ik_joints
            bin_qpos[7] = GRIPPER_CLOSE
            bin_qpos[8] = GRIPPER_CLOSE

            print(f"    Moving to bin at [{bin_target[0]:.2f}, {bin_target[1]:.2f}, {bin_target[2]:.2f}]")

            def move_to_bin():
                # Lift up first
                for _ in self._interpolate_to_target(lift_qpos, steps=30):
                    yield
                # Move to bin using IK-solved position
                for _ in self._interpolate_to_target(bin_qpos, steps=40):
                    yield

            return move_to_bin()

        elif step_type == "place":
            # Open gripper and drop object into bin
            self.gripper_target = GRIPPER_OPEN

            if self.held_object and self.held_object in self.object_body_ids:
                # Drop object into bin
                body_id = self.object_body_ids[self.held_object]
                import random
                bin_x = BIN_POS[0] + random.uniform(-0.05, 0.05)
                bin_y = BIN_POS[1] + random.uniform(-0.05, 0.05)
                bin_z = 0.35  # Inside the bin
                self.model.body_pos[body_id] = [bin_x, bin_y, bin_z]
                mujoco.mj_forward(self.model, self.data)
                print(f"    Dropped {self.held_object} in bin at [{bin_x:.2f}, {bin_y:.2f}, {bin_z:.2f}]")
                self.held_object = None

            current = self._get_arm_qpos()
            target = current.copy()
            target[7] = GRIPPER_OPEN
            target[8] = GRIPPER_OPEN
            return self._interpolate_to_target(target, steps=20)

        elif step_type == "done":
            # Return to home position
            print("    Returning to home position")
            home_qpos = np.array(HOME_JOINTS + [GRIPPER_OPEN, GRIPPER_OPEN])
            return self._interpolate_to_target(home_qpos, steps=50)

        return None

    def has_pending_steps(self):
        with self.lock:
            return len(self.step_queue) > 0


async def websocket_listener(url: str, order_id: str, controller: RobotController) -> bool:
    """Connect to backend WebSocket and receive robot steps. Returns True when order_complete, False on error/disconnect."""
    ws_url = f"{url}/ws/{order_id}"
    print(f"Connecting to WebSocket: {ws_url}")

    try:
        async with websockets.connect(ws_url) as ws:
            print("Connected! Waiting for robot steps...")
            async for message in ws:
                try:
                    msg = json.loads(message)
                    if msg.get("type") == "order_complete":
                        print("Order complete.")
                        return True
                    if msg.get("type") == "robot_step":
                        step = msg.get("step", "")
                        item_id = msg.get("item_id", "")
                        message_text = msg.get("message", "")
                        print(f"Received step: {step} - {item_id}: {message_text}")
                        controller.add_step(step, item_id, message_text)
                except json.JSONDecodeError:
                    print(f"Invalid JSON: {message}")
        return False
    except Exception as e:
        print(f"WebSocket error: {e}")
        return False


def _capture_wrist_image(model, data):
    """Render one frame from hand_camera and return PNG bytes, or None on failure."""
    if model is None or data is None:
        return None
    try:
        # hand_camera resolution in scene is 640x480
        width, height = 640, 480
        renderer = mujoco.Renderer(model, height=height, width=width)
        renderer.update_scene(data, camera="hand_camera")
        pixels = renderer.render()
        if pixels is None or pixels.size == 0:
            return None
        if Image is None:
            return None
        img = Image.fromarray(pixels)
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()
    except Exception as e:
        print(f"Wrist capture failed: {e}")
        return None


async def _send_wrist_image_to_backend(http_base: str, order_id: str, image_bytes: bytes) -> bool:
    """POST wrist image to backend. Returns True on success."""
    if not image_bytes or not http_base or not order_id:
        return False
    if httpx is None:
        print("httpx not installed; skipping wrist image upload")
        return False
    url = f"{http_base.rstrip('/')}/api/orders/{order_id}/wrist_image"
    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            r = await client.post(url, content=image_bytes, headers={"Content-Type": "image/png"})
            if r.status_code == 200:
                return True
            print(f"Wrist image upload failed: {r.status_code}")
            return False
    except Exception as e:
        print(f"Wrist image upload error: {e}")
        return False


def _capture_viewer_frame(model, data, camera="default"):
    """Render one frame from the specified camera and return base64-encoded PNG string."""
    if model is None or data is None:
        return None
    try:
        import base64
        width, height = 640, 480
        renderer = mujoco.Renderer(model, height=height, width=width)
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        if pixels is None or pixels.size == 0:
            return None
        if Image is None:
            return None
        img = Image.fromarray(pixels)
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return base64.b64encode(buf.getvalue()).decode('utf-8')
    except Exception as e:
        return None


async def _send_frame_to_backend(http_base: str, frame_data: str) -> bool:
    """POST simulation frame (base64 image) to backend for broadcasting to viewers."""
    if not frame_data or not http_base:
        return False
    if httpx is None:
        return False
    url = f"{http_base.rstrip('/')}/api/sim/frame"
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            r = await client.post(url, content=frame_data.encode('utf-8'))
            return r.status_code == 200
    except Exception:
        return False


async def wait_for_orders_loop(url: str, controller: RobotController):
    """Connect to /ws/sim and run each order when the backend sends new_order."""
    sim_ws_url = f"{url}/ws/sim"
    print(f"Connecting to sim channel: {sim_ws_url}")
    while True:
        try:
            async with websockets.connect(sim_ws_url) as ws_sim:
                print("Sim connected. Submit a task on the frontend to run the robot.")
                async for message in ws_sim:
                    try:
                        msg = json.loads(message)
                        if msg.get("type") == "new_order":
                            order_id = msg.get("order_id")
                            if order_id:
                                print(f"\n--- New order: {order_id} ---")
                                # Capture wrist camera and send to backend for vision-based parsing
                                http_base = url.replace("ws://", "http://").replace("wss://", "https://")
                                image_bytes = _capture_wrist_image(controller.model, controller.data)
                                if image_bytes:
                                    ok = await _send_wrist_image_to_backend(http_base, order_id, image_bytes)
                                    if ok:
                                        print("Wrist image sent to backend.")
                                await websocket_listener(url, order_id, controller)
                                # Don't reset - objects stay in bin after being dropped
                    except json.JSONDecodeError:
                        pass
        except Exception as e:
            print(f"Sim channel error: {e}")
        await asyncio.sleep(2)


def run_simulation(controller: RobotController, model, data, http_base: str = None, enable_web_streaming: bool = False):
    """Run the MuJoCo simulation with real-time robot control."""
    print("\nLaunching MuJoCo viewer...")
    print("The robot will move when steps are received from the backend.\n")
    if enable_web_streaming:
        print("Web streaming enabled - frames will be sent to frontend viewers.\n")

    current_animation = None
    frame_counter = 0
    last_frame_time = time.time()

    # Use a simple loop with mj_step and render
    import platform

    if platform.system() == "Darwin":
        # macOS: use glfw directly for rendering
        try:
            import glfw

            # Initialize GLFW
            if not glfw.init():
                raise RuntimeError("Failed to initialize GLFW")

            # Create window
            window = glfw.create_window(1200, 900, "Cleanup Room Simulation", None, None)
            if not window:
                glfw.terminate()
                raise RuntimeError("Failed to create GLFW window")

            glfw.make_context_current(window)
            glfw.swap_interval(1)

            # Create scene and context for rendering
            scene = mujoco.MjvScene(model, maxgeom=10000)
            context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
            camera = mujoco.MjvCamera()
            option = mujoco.MjvOption()

            # Default to scene view (free camera) so users can see the robot moving
            # Hand camera can be selected in the viewer's rendering tab
            camera.lookat[:] = [0.0, 0.0, 0.35]
            camera.distance = 2.2
            camera.azimuth = 135
            camera.elevation = -25

            # Mouse state for camera control
            last_x, last_y = 0, 0
            button_left = False
            button_right = False

            def mouse_button_callback(window, button, action, mods):
                nonlocal button_left, button_right
                if button == glfw.MOUSE_BUTTON_LEFT:
                    button_left = (action == glfw.PRESS)
                elif button == glfw.MOUSE_BUTTON_RIGHT:
                    button_right = (action == glfw.PRESS)

            def cursor_pos_callback(window, xpos, ypos):
                nonlocal last_x, last_y
                dx = xpos - last_x
                dy = ypos - last_y
                last_x, last_y = xpos, ypos

                if button_left:
                    camera.azimuth += dx * 0.5
                    camera.elevation = max(-90, min(90, camera.elevation - dy * 0.5))
                elif button_right:
                    camera.lookat[0] -= dx * 0.01
                    camera.lookat[1] += dy * 0.01

            def scroll_callback(window, xoffset, yoffset):
                camera.distance = max(0.5, camera.distance - yoffset * 0.2)

            def key_callback(window, key, scancode, action, mods):
                if action == glfw.PRESS:
                    if key == glfw.KEY_R:
                        controller.reset_objects()
                        print("Scene reset!")
                    elif key == glfw.KEY_H:
                        # Return arm to home position
                        controller.add_step("done", "", "Home")
                        print("Returning to home...")

            glfw.set_mouse_button_callback(window, mouse_button_callback)
            glfw.set_cursor_pos_callback(window, cursor_pos_callback)
            glfw.set_scroll_callback(window, scroll_callback)
            glfw.set_key_callback(window, key_callback)

            print("Controls: Left-drag=rotate, Right-drag=pan, Scroll=zoom, R=reset scene, H=home")

            while not glfw.window_should_close(window):
                step_start = time.time()

                # Process animation frames
                if current_animation is not None:
                    try:
                        next(current_animation)
                    except StopIteration:
                        current_animation = None
                elif controller.has_pending_steps():
                    current_animation = controller.process_step()

                # Update held object position to follow gripper
                controller.update_held_object()

                # Step physics
                mujoco.mj_step(model, data)

                # Get framebuffer size
                width, height = glfw.get_framebuffer_size(window)
                viewport = mujoco.MjrRect(0, 0, width, height)

                # Update scene and render
                mujoco.mjv_updateScene(model, data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
                mujoco.mjr_render(viewport, scene, context)

                glfw.swap_buffers(window)
                glfw.poll_events()

                # Frame rate control
                dt = model.opt.timestep
                elapsed = time.time() - step_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

            glfw.terminate()

        except Exception as e:
            print(f"GLFW error: {e}")
            print("\nFallback: Running without viewer (headless mode)")
            _run_headless(controller, model, data)
    else:
        # Non-macOS: use launch_passive
        with mujoco.viewer.launch_passive(model, data) as viewer:
            hand_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "hand_camera")
            if hand_cam_id >= 0:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = hand_cam_id
            else:
                viewer.cam.lookat[:] = [0.0, 0.0, 0.35]
                viewer.cam.distance = 2.5
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -25

            while viewer.is_running():
                step_start = time.time()

                if current_animation is not None:
                    try:
                        next(current_animation)
                    except StopIteration:
                        current_animation = None
                elif controller.has_pending_steps():
                    current_animation = controller.process_step()

                controller.update_held_object()
                mujoco.mj_step(model, data)
                viewer.sync()

                # Send frames to web viewers (throttled to ~10 FPS)
                if enable_web_streaming and http_base:
                    frame_counter += 1
                    if frame_counter % 5 == 0:  # Send every 5th frame
                        current_time = time.time()
                        if current_time - last_frame_time >= 0.1:  # Max 10 FPS
                            frame_data = _capture_viewer_frame(model, data, camera="default")
                            if frame_data:
                                # Send in background thread to avoid blocking
                                threading.Thread(target=lambda: asyncio.run(_send_frame_to_backend(http_base, frame_data)), daemon=True).start()
                            last_frame_time = current_time

                dt = model.opt.timestep
                elapsed = time.time() - step_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

    print("\nViewer closed.")


def _run_headless(controller, model, data):
    """Run simulation without viewer (for testing)."""
    print("Running in headless mode...")
    current_animation = None

    while controller.has_pending_steps() or current_animation is not None:
        if current_animation is not None:
            try:
                next(current_animation)
            except StopIteration:
                current_animation = None
        elif controller.has_pending_steps():
            current_animation = controller.process_step()

        mujoco.mj_step(model, data)
        time.sleep(model.opt.timestep)

    print("Headless simulation complete.")


def main():
    parser = argparse.ArgumentParser(description="MuJoCo simulation client for Vultr backend")
    parser.add_argument("order_id", nargs="?", help="Order ID to subscribe to (or use --wait to get orders from frontend)")
    parser.add_argument("--backend-url", default="ws://localhost:8000", help="Backend WebSocket URL")
    parser.add_argument("--demo", action="store_true", help="Run demo mode without backend connection")
    parser.add_argument("--wait", action="store_true", help="Connect to /ws/sim; run each order when user submits on frontend")
    parser.add_argument("--enable-web-viewer", action="store_true", help="Stream simulation frames to web frontend")
    args = parser.parse_args()

    # Convert WebSocket URL to HTTP base URL for frame streaming
    http_base = args.backend_url.replace("ws://", "http://").replace("wss://", "https://")

    print("Loading MuJoCo scene...")
    model, data = load_scene_with_table()

    if model is None:
        print("ERROR: Could not load scene. Make sure object_sim is cloned.")
        sys.exit(1)

    print(f"Loaded: {model.nbody} bodies, {model.ngeom} geoms")

    # Create controller
    controller = RobotController(model, data)

    if args.demo or (not args.order_id and not args.wait):
        # Demo mode: add some test steps
        print("\n=== DEMO MODE ===")
        print("Adding test steps...")

        # Pick up elephant and another object for demo
        test_objects = ["elephant", "duck", "banana"]
        picked = 0
        for obj in test_objects:
            if obj in OBJECT_POSITIONS and picked < 2:
                controller.add_step("move_to_pick", obj, f"Moving to {obj}")
                controller.add_step("pick", obj, f"Picking {obj}")
                controller.add_step("move_to_delivery", obj, f"Taking {obj} to bin")
                controller.add_step("place", obj, f"Placed {obj}")
                picked += 1

        if picked > 0:
            controller.add_step("done", "", "Done!")

        run_simulation(controller, model, data, http_base, args.enable_web_viewer)
    elif args.wait:
        # Wait mode: scene is already loaded; listen for orders from frontend
        print("\n=== WAIT MODE ===")
        print("Scene loaded. Submit a task on the frontend to run the robot.")
        print(f"Backend: {args.backend_url}\n")

        def ws_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(wait_for_orders_loop(args.backend_url, controller))

        thread = threading.Thread(target=ws_thread, daemon=True)
        thread.start()

        run_simulation(controller, model, data, http_base, args.enable_web_viewer)
    else:
        # Live mode: single order
        print(f"\n=== LIVE MODE ===")
        print(f"Order ID: {args.order_id}")
        print(f"Backend: {args.backend_url}")

        def ws_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(
                websocket_listener(args.backend_url, args.order_id, controller)
            )

        thread = threading.Thread(target=ws_thread, daemon=True)
        thread.start()

        run_simulation(controller, model, data, http_base, args.enable_web_viewer)


if __name__ == "__main__":
    main()
