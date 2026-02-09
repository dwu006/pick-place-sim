import json
from typing import Dict, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()

# Global connection registry: job_id -> set of websockets
connections: Dict[str, Set[WebSocket]] = {}


async def broadcast(job_id: str, message: dict):
    """Send a message to all clients watching a job."""
    if job_id not in connections:
        return
    dead = set()
    for ws in connections[job_id]:
        try:
            await ws.send_text(json.dumps(message))
        except Exception:
            dead.add(ws)
    connections[job_id] -= dead
    if not connections[job_id]:
        del connections[job_id]


@router.websocket("/ws/{job_id}")
async def job_websocket(websocket: WebSocket, job_id: str):
    await websocket.accept()
    connections.setdefault(job_id, set()).add(websocket)
    try:
        while True:
            # Keep connection alive, client can send pings
            await websocket.receive_text()
    except WebSocketDisconnect:
        connections.get(job_id, set()).discard(websocket)
