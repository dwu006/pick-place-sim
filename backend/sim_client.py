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

# Pre-computed configurations for different approach angles and phases
# Format: [J1, J2, J3, J4, J5, J6, J7, finger1, finger2]
PICK_CONFIGS = {
    # Home/raised position - arm up and safe
    "home": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],

    # Approach configurations (arm extended but not fully down)
    "approach_front": [0.0, 0.2, 0.0, -1.8, 0.0, 2.0, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],
    "approach_right": [0.7, 0.2, 0.0, -1.8, 0.0, 2.0, 0.5, GRIPPER_OPEN, GRIPPER_OPEN],
    "approach_left": [-0.7, 0.2, 0.0, -1.8, 0.0, 2.0, 1.0, GRIPPER_OPEN, GRIPPER_OPEN],
    "approach_back": [3.14, 0.2, 0.0, -1.8, 0.0, 2.0, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],

    # Pick configurations (arm extended down to table level)
    "pick_front": [0.0, 0.6, 0.0, -1.4, 0.0, 2.0, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],
    "pick_right": [0.7, 0.6, 0.0, -1.4, 0.0, 2.0, 0.5, GRIPPER_OPEN, GRIPPER_OPEN],
    "pick_left": [-0.7, 0.6, 0.0, -1.4, 0.0, 2.0, 1.0, GRIPPER_OPEN, GRIPPER_OPEN],
    "pick_back": [3.14, 0.6, 0.0, -1.4, 0.0, 2.0, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],

    # Bin position (0.5, -0.35) - angle ~-35 degrees from front
    "bin_approach": [-0.6, 0.2, 0.0, -1.8, 0.0, 2.0, 0.6, GRIPPER_CLOSE, GRIPPER_CLOSE],
    "bin_drop": [-0.6, 0.5, 0.0, -1.5, 0.0, 2.0, 0.6, GRIPPER_CLOSE, GRIPPER_CLOSE],
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
        """Set arm joint positions directly."""
        # Franka has 7 arm joints + 2 gripper joints
        for i, q in enumerate(qpos[:7]):
            self.data.qpos[i] = q
        # Gripper
        if len(qpos) > 7:
            self.data.qpos[7] = qpos[7]
            self.data.qpos[8] = qpos[8]
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

    def _compute_ik_for_position(self, target_pos, phase="pick", for_bin=False):
        """
        Compute joint angles to reach a target position.

        Args:
            target_pos: [x, y, z] target position
            phase: "approach" (above object) or "pick" (at object level)
            for_bin: True if moving to bin

        Uses analytical heuristics based on Franka Panda kinematics:
        - J1: Base rotation (points arm toward target)
        - J2: Shoulder flexion (controls reach distance)
        - J3: Additional shoulder rotation
        - J4: Elbow (main reach control, always negative)
        - J5: Wrist flexion
        - J6: Wrist rotation (keeps gripper down)
        - J7: Gripper rotation (aligns fingers)
        """
        import math

        x, y, z = target_pos

        # Calculate angle and distance to target (in xy plane)
        angle = math.atan2(y, x)
        reach = math.sqrt(x**2 + y**2)

        # Clamp reach to feasible range (0.35 to 0.7m for reliable picking)
        reach = max(0.35, min(0.70, reach))

        if for_bin:
            # Bin is at approximately (0.5, -0.35) -> angle ~ -0.61 rad
            if phase == "approach":
                qpos = np.array(PICK_CONFIGS["bin_approach"]).copy()
            else:
                qpos = np.array(PICK_CONFIGS["bin_drop"]).copy()
            qpos[7] = self.gripper_target
            qpos[8] = self.gripper_target
            return qpos

        # Compute joint angles analytically
        qpos = np.zeros(9)

        # J1: Base rotation - point toward target
        qpos[0] = angle

        # Normalize reach to 0-1 range for interpolation
        reach_norm = (reach - 0.35) / 0.35  # 0 at 0.35m, 1 at 0.7m
        reach_norm = max(0.0, min(1.0, reach_norm))

        if phase == "approach":
            # Approach: arm extended but raised
            qpos[1] = 0.1 + reach_norm * 0.2      # Shoulder: 0.1 to 0.3
            qpos[2] = 0.0                          # Shoulder rotation
            qpos[3] = -1.9 + reach_norm * 0.2     # Elbow: -1.9 to -1.7
            qpos[4] = 0.0                          # Forearm rotation
            qpos[5] = 2.0 - reach_norm * 0.2      # Wrist: 2.0 to 1.8
            qpos[6] = 0.785                        # Gripper rotation
        else:
            # Pick: arm extended and lowered to table
            qpos[1] = 0.5 + reach_norm * 0.3      # Shoulder: 0.5 to 0.8 (more forward)
            qpos[2] = 0.0                          # Shoulder rotation
            qpos[3] = -1.5 + reach_norm * 0.3     # Elbow: -1.5 to -1.2 (more extended)
            qpos[4] = 0.0                          # Forearm rotation
            qpos[5] = 2.0 - reach_norm * 0.3      # Wrist: 2.0 to 1.7
            qpos[6] = 0.785                        # Gripper rotation

        # Adjust J7 (gripper rotation) based on approach angle for better grasp
        # Keep gripper roughly aligned with object
        qpos[6] = 0.785 - angle * 0.1

        # Clamp to joint limits
        joint_limits = [
            (-2.8973, 2.8973),   # J1
            (-1.7628, 1.7628),   # J2
            (-2.8973, 2.8973),   # J3
            (-3.0718, -0.0698),  # J4
            (-2.8973, 2.8973),   # J5
            (-0.0175, 3.7525),   # J6
            (-2.8973, 2.8973),   # J7
        ]
        for i in range(7):
            qpos[i] = max(joint_limits[i][0], min(joint_limits[i][1], qpos[i]))

        # Set gripper
        qpos[7] = self.gripper_target
        qpos[8] = self.gripper_target

        return qpos

    def update_held_object(self):
        """Update position of held object to follow gripper."""
        if self.held_object and self.held_object in self.object_body_ids:
            body_id = self.object_body_ids[self.held_object]
            # Get end effector position (approximate from last link)
            # For Franka, link8 is the end effector
            ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link8")
            if ee_body_id >= 0:
                ee_pos = self.data.xpos[ee_body_id].copy()
                ee_pos[2] -= 0.05  # offset below gripper
                self.model.body_pos[body_id] = ee_pos

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
        with self.lock:
            if not self.step_queue:
                return None
            step = self.step_queue.pop(0)

        step_type = step["step"]
        item_id = step["item_id"]

        print(f"  Executing: {step_type} - {item_id}")

        if step_type == "move_to_pick":
            # Move to object position - two-phase: approach (above) then lower to pick height
            if item_id in OBJECT_POSITIONS:
                target_pos = OBJECT_POSITIONS[item_id].copy()
                print(f"    Moving to {item_id} at [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")

                self.gripper_target = GRIPPER_OPEN

                # Phase 1: Approach (move above the object)
                approach_qpos = self._compute_ik_for_position(target_pos, phase="approach")
                approach_qpos[7] = GRIPPER_OPEN
                approach_qpos[8] = GRIPPER_OPEN

                # Phase 2: Lower to pick position
                pick_qpos = self._compute_ik_for_position(target_pos, phase="pick")
                pick_qpos[7] = GRIPPER_OPEN
                pick_qpos[8] = GRIPPER_OPEN

                # Return a generator that does both phases
                def two_phase_move():
                    # Phase 1: Move to approach position
                    for _ in self._interpolate_to_target(approach_qpos, steps=40):
                        yield
                    # Phase 2: Lower to pick position
                    for _ in self._interpolate_to_target(pick_qpos, steps=30):
                        yield

                return two_phase_move()
            else:
                print(f"    WARNING: Object {item_id} not found in positions!")
                return None

        elif step_type == "pick":
            # Close gripper and attach object
            self.gripper_target = GRIPPER_CLOSE
            if item_id in self.object_body_ids:
                self.held_object = item_id
                print(f"    Grasping {item_id}")
            current = self._get_arm_qpos()
            target = current.copy()
            target[7] = GRIPPER_CLOSE
            target[8] = GRIPPER_CLOSE
            return self._interpolate_to_target(target, steps=25)

        elif step_type == "move_to_delivery":
            # Move to bin - two-phase: lift then move to bin
            self.gripper_target = GRIPPER_CLOSE

            # Get current position for lift
            current = self._get_arm_qpos()

            # Phase 1: Lift (raise shoulder)
            lift_qpos = current.copy()
            lift_qpos[1] = 0.0  # Raise shoulder
            lift_qpos[3] = -2.0  # Retract elbow
            lift_qpos[7] = GRIPPER_CLOSE
            lift_qpos[8] = GRIPPER_CLOSE

            # Phase 2: Move to bin approach
            bin_approach_qpos = self._compute_ik_for_position(BIN_POS, phase="approach", for_bin=True)
            bin_approach_qpos[7] = GRIPPER_CLOSE
            bin_approach_qpos[8] = GRIPPER_CLOSE

            # Phase 3: Lower to bin
            bin_drop_qpos = self._compute_ik_for_position(BIN_POS, phase="pick", for_bin=True)
            bin_drop_qpos[7] = GRIPPER_CLOSE
            bin_drop_qpos[8] = GRIPPER_CLOSE

            def move_to_bin():
                # Lift object
                for _ in self._interpolate_to_target(lift_qpos, steps=25):
                    yield
                # Move to bin approach
                for _ in self._interpolate_to_target(bin_approach_qpos, steps=35):
                    yield
                # Lower to bin
                for _ in self._interpolate_to_target(bin_drop_qpos, steps=20):
                    yield

            return move_to_bin()

        elif step_type == "place":
            # Open gripper and release object into bin
            self.gripper_target = GRIPPER_OPEN
            if self.held_object and self.held_object in self.object_body_ids:
                # Move object to bin position
                body_id = self.object_body_ids[self.held_object]
                # Set object position to bin (with some randomness so they don't stack perfectly)
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
            return self._interpolate_to_target(np.array(HOME_QPOS), steps=60)

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
                                controller.reset_objects()
                    except json.JSONDecodeError:
                        pass
        except Exception as e:
            print(f"Sim channel error: {e}")
        await asyncio.sleep(2)


def run_simulation(controller: RobotController, model, data):
    """Run the MuJoCo simulation with real-time robot control."""
    print("\nLaunching MuJoCo viewer...")
    print("The robot will move when steps are received from the backend.\n")

    current_animation = None

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

            glfw.set_mouse_button_callback(window, mouse_button_callback)
            glfw.set_cursor_pos_callback(window, cursor_pos_callback)
            glfw.set_scroll_callback(window, scroll_callback)

            print("Controls: Left-drag=rotate, Right-drag=pan, Scroll=zoom")

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
    args = parser.parse_args()

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

        run_simulation(controller, model, data)
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

        run_simulation(controller, model, data)
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

        run_simulation(controller, model, data)


if __name__ == "__main__":
    main()
