import json
from typing import Dict, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()

connections: Dict[str, Set[WebSocket]] = {}
sim_subscribers: Set[WebSocket] = set()


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


async def broadcast_to_sim(order_id: str):
    """Notify all connected sim clients that a new order is ready to run."""
    dead = set()
    for ws in sim_subscribers:
        try:
            await ws.send_text(json.dumps({"type": "new_order", "order_id": order_id}))
        except Exception:
            dead.add(ws)
    for ws in dead:
        sim_subscribers.discard(ws)


@router.websocket("/ws/{order_id}")
async def order_websocket(websocket: WebSocket, order_id: str):
    await websocket.accept()
    connections.setdefault(order_id, set()).add(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        connections.get(order_id, set()).discard(websocket)


@router.websocket("/ws/sim")
async def sim_websocket(websocket: WebSocket):
    """Channel for sim clients waiting for orders. Backend sends new_order when user submits a task."""
    await websocket.accept()
    sim_subscribers.add(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        sim_subscribers.discard(websocket)
