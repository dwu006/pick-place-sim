"""Spawn the MuJoCo sim client (for frontend-triggered start or per-order spawn)."""
import os
import subprocess
import sys
from typing import Optional

_sim_waiting_spawned = False


def spawn_sim_client(order_id: Optional[str] = None) -> bool:
    """
    Spawn sim_client in a new process.
    - order_id: subscribe to this order (legacy per-order spawn).
    - order_id is None: run with --wait so it connects to /ws/sim and waits for orders.
    Returns True if spawn was attempted, False if skipped.
    """
    try:
        backend_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.dirname(backend_dir)
        sim_client_script = os.path.join(backend_dir, "sim_client.py")
        if not os.path.isfile(sim_client_script):
            return False
        global _sim_waiting_spawned
        if order_id is None:
            if _sim_waiting_spawned:
                return True  # Already started once
            _sim_waiting_spawned = True
        args = [sys.executable, sim_client_script, "--backend-url", "ws://127.0.0.1:8000"]
        if order_id:
            args.insert(2, order_id)  # [python, sim_client.py, order_id, --backend-url, ...]
        else:
            args.insert(2, "--wait")  # [python, sim_client.py, --wait, --backend-url, ...]
        kwargs = {"cwd": repo_root, "env": {**os.environ}}
        if sys.platform == "win32":
            kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE
        else:
            kwargs["start_new_session"] = True
        subprocess.Popen(args, **kwargs)
        return True
    except Exception:
        return False
