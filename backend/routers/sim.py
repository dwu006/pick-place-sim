from fastapi import APIRouter

from spawn_sim import spawn_sim_client

router = APIRouter()


@router.post("/start")
async def start_sim():
    """Start the MuJoCo sim client (waiting mode). Scene loads once; backend pushes order_id when user submits a task."""
    ok = spawn_sim_client(order_id=None)
    return {"status": "started" if ok else "already_running", "message": "Sim client starting (connects to /ws/sim for orders)."}
