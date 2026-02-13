import json
from typing import Dict, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()

connections: Dict[str, Set[WebSocket]] = {}


async def broadcast(order_id: str, message: dict):
    """Send a message to all clients watching an order."""
    if order_id not in connections:
        return
    dead = set()
    for ws in connections[order_id]:
        try:
            await ws.send_text(json.dumps(message))
        except Exception:
            dead.add(ws)
    connections[order_id] -= dead
    if not connections[order_id]:
        del connections[order_id]


@router.websocket("/ws/{order_id}")
async def order_websocket(websocket: WebSocket, order_id: str):
    await websocket.accept()
    connections.setdefault(order_id, set()).add(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        connections.get(order_id, set()).discard(websocket)
