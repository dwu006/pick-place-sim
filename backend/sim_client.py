"""
Real-time MuJoCo simulation client that connects to Vultr backend via WebSocket.
Receives robot steps and moves the Franka arm in real-time.

Usage:
  python sim_client.py <order_id> [--backend-url ws://YOUR_VULTR_IP:8000]

Example:
  python sim_client.py abc123 --backend-url ws://149.28.xx.xx:8000
"""
import argparse
import asyncio
import json
import os
import sys
import time
import threading
from typing import Optional

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

# Default object positions from _cleanup_room_objects.xml
# These are the exact spawn positions
DEFAULT_OBJECT_POSITIONS = {
    "banana": [-0.405, -0.498, 0.323],
    "duck": [0.706, -0.744, 0.348],
    "phone": [-0.506, 0.010, 0.325],
    "elephant": [-0.503, 0.161, 0.350],
    "eyeglasses": [0.550, 0.357, 0.322],
    "flute": [-0.288, -0.620, 0.308],
    "gamecontroller": [-0.733, -0.726, 0.325],
    "headphones": [0.625, 0.187, 0.329],
    "mouse": [0.368, -0.818, 0.320],
    "piggybank": [-0.490, -0.379, 0.368],
    "pyramidlarge": [-0.756, -0.481, 0.350],
    "stanfordbunny": [-0.419, 0.786, 0.363],
    "train": [-0.592, 0.412, 0.342],
    "watch": [-0.606, -0.217, 0.341],
    "airplane": [0.617, 0.497, 0.325],
    "alarmclock": [0.746, -0.074, 0.354],
    "camera": [0.152, 0.716, 0.334],
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

# Predefined pick configurations - tuned to reach z ~ 0.35 table surface
# Franka Panda joint limits: J1[-2.9,2.9], J2[-1.76,1.76], J3[-2.9,2.9], J4[-3.07,-0.07], J5[-2.9,2.9], J6[-0.02,3.75], J7[-2.9,2.9]
PICK_CONFIGS = {
    # Reaching forward and down (for objects in front)
    "forward": [0.0, 0.5, 0.0, -2.0, 0.0, 2.5, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],
    # Reaching to the right
    "right": [0.8, 0.6, 0.0, -1.8, 0.0, 2.4, 0.5, GRIPPER_OPEN, GRIPPER_OPEN],
    # Reaching to the left
    "left": [-0.8, 0.6, 0.0, -1.8, 0.0, 2.4, 1.0, GRIPPER_OPEN, GRIPPER_OPEN],
    # Reaching backward-right
    "back_right": [2.0, 0.4, 0.0, -2.2, 0.0, 2.6, 0.0, GRIPPER_OPEN, GRIPPER_OPEN],
    # Reaching backward-left
    "back_left": [-2.0, 0.4, 0.0, -2.2, 0.0, 2.6, 1.5, GRIPPER_OPEN, GRIPPER_OPEN],
    # For bin drop (bin is at 0.5, -0.35)
    "bin": [0.6, 0.3, 0.0, -2.2, 0.0, 2.5, 0.6, GRIPPER_CLOSE, GRIPPER_CLOSE],
    # Home/raised position
    "home": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, GRIPPER_OPEN, GRIPPER_OPEN],
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

        # Step simulation once to compute positions
        mujoco.mj_forward(model, data)

        # Find object body positions
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

    def _compute_ik_for_position(self, target_pos, for_bin=False):
        """
        Compute joint angles to reach a target position.
        Uses predefined configurations based on direction, then adjusts.
        """
        import math

        x, y, z = target_pos

        if for_bin:
            qpos = np.array(PICK_CONFIGS["bin"]).copy()
            qpos[7] = self.gripper_target
            qpos[8] = self.gripper_target
            return qpos

        # Calculate angle to target
        angle = math.atan2(y, x)
        reach = math.sqrt(x**2 + y**2)

        # Select base config based on angle (in radians)
        # Forward: -45 to 45 deg, Right: -135 to -45, Left: 45 to 135, Back: else
        angle_deg = math.degrees(angle)

        if -45 <= angle_deg <= 45:
            base_config = "forward"
        elif 45 < angle_deg <= 135:
            base_config = "left"
        elif -135 <= angle_deg < -45:
            base_config = "right"
        elif angle_deg > 135:
            base_config = "back_left"
        else:
            base_config = "back_right"

        qpos = np.array(PICK_CONFIGS[base_config]).copy()

        # Set base rotation (J0) to point at target
        qpos[0] = angle

        # Adjust for reach distance (objects are 0.4-0.8m away)
        reach_factor = (reach - 0.5) / 0.3
        reach_factor = max(-1.0, min(1.0, reach_factor))

        # Shoulder (J1): lean forward more for farther objects
        qpos[1] += reach_factor * 0.25

        # Elbow (J3): extend more for farther objects
        qpos[3] -= reach_factor * 0.15

        # Wrist (J5): adjust angle
        qpos[5] += reach_factor * 0.1

        # Keep gripper pointing down - adjust J6 for hand orientation
        qpos[6] = 0.785 + angle * 0.2

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
            # Move to object position - two-phase: approach then lower
            if item_id in OBJECT_POSITIONS:
                target_pos = OBJECT_POSITIONS[item_id].copy()
                print(f"    Moving to {item_id} at [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")

                # Compute base configuration
                target_qpos = self._compute_ik_for_position(target_pos)
                self.gripper_target = GRIPPER_OPEN
                target_qpos[7] = GRIPPER_OPEN
                target_qpos[8] = GRIPPER_OPEN

                # Lower the arm more (adjust elbow and wrist to reach down)
                target_qpos[1] += 0.3   # shoulder down
                target_qpos[3] += 0.4   # elbow more bent (reaches lower)
                target_qpos[5] += 0.2   # wrist adjustment

                return self._interpolate_to_target(target_qpos, steps=70)

        elif step_type == "pick":
            # Close gripper and attach object
            self.gripper_target = GRIPPER_CLOSE
            if item_id in self.object_body_ids:
                self.held_object = item_id
            current = self._get_arm_qpos()
            target = current.copy()
            target[7] = GRIPPER_CLOSE
            target[8] = GRIPPER_CLOSE
            return self._interpolate_to_target(target, steps=30)

        elif step_type == "move_to_delivery":
            # Move to bin
            target_qpos = self._compute_ik_for_position(BIN_POS, for_bin=True)
            target_qpos[7] = GRIPPER_CLOSE
            target_qpos[8] = GRIPPER_CLOSE
            return self._interpolate_to_target(target_qpos, steps=60)

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
            return self._interpolate_to_target(target, steps=30)

        elif step_type == "done":
            # Return to home
            return self._interpolate_to_target(HOME_QPOS, steps=80)

        return None

    def has_pending_steps(self):
        with self.lock:
            return len(self.step_queue) > 0


async def websocket_listener(url: str, order_id: str, controller: RobotController):
    """Connect to backend WebSocket and receive robot steps."""
    ws_url = f"{url}/ws/{order_id}"
    print(f"Connecting to WebSocket: {ws_url}")

    try:
        async with websockets.connect(ws_url) as ws:
            print("Connected! Waiting for robot steps...")
            async for message in ws:
                try:
                    msg = json.loads(message)
                    if msg.get("type") == "robot_step":
                        step = msg.get("step", "")
                        item_id = msg.get("item_id", "")
                        message_text = msg.get("message", "")
                        print(f"Received step: {step} - {item_id}: {message_text}")
                        controller.add_step(step, item_id, message_text)
                except json.JSONDecodeError:
                    print(f"Invalid JSON: {message}")
    except Exception as e:
        print(f"WebSocket error: {e}")


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

            # Set camera
            camera.lookat[:] = [0.0, 0.0, 0.35]
            camera.distance = 2.5
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
    parser.add_argument("order_id", nargs="?", help="Order ID to subscribe to (or 'demo' for local demo)")
    parser.add_argument("--backend-url", default="ws://localhost:8000", help="Backend WebSocket URL")
    parser.add_argument("--demo", action="store_true", help="Run demo mode without backend connection")
    args = parser.parse_args()

    print("Loading MuJoCo scene...")
    model, data = load_scene_with_table()

    if model is None:
        print("ERROR: Could not load scene. Make sure object_sim is cloned.")
        sys.exit(1)

    print(f"Loaded: {model.nbody} bodies, {model.ngeom} geoms")

    # Create controller
    controller = RobotController(model, data)

    if args.demo or not args.order_id:
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
    else:
        # Live mode: connect to backend
        print(f"\n=== LIVE MODE ===")
        print(f"Order ID: {args.order_id}")
        print(f"Backend: {args.backend_url}")

        # Start WebSocket listener in background thread
        def ws_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(
                websocket_listener(args.backend_url, args.order_id, controller)
            )

        thread = threading.Thread(target=ws_thread, daemon=True)
        thread.start()

        # Run simulation in main thread
        run_simulation(controller, model, data)


if __name__ == "__main__":
    main()
