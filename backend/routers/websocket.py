import json
import logging
from typing import Dict, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()
logger = logging.getLogger(__name__)

connections: Dict[str, Set[WebSocket]] = {}
sim_subscribers: Set[WebSocket] = set()
viewer_subscribers: Set[WebSocket] = set()  # Frontends watching the simulation visualization


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
    logger.info(f"broadcast_to_sim called for order {order_id}, sim_subscribers count: {len(sim_subscribers)}")
    dead = set()
    for ws in sim_subscribers:
        try:
            logger.info(f"Sending new_order to sim client: {order_id}")
            await ws.send_text(json.dumps({"type": "new_order", "order_id": order_id}))
        except Exception as e:
            logger.error(f"Failed to send to sim client: {e}")
            dead.add(ws)
    for ws in dead:
        sim_subscribers.discard(ws)


async def broadcast_frame(frame_data: str):
    """Broadcast a simulation frame (base64 image) to all viewer subscribers."""
    dead = set()
    for ws in viewer_subscribers:
        try:
            await ws.send_text(json.dumps({"type": "frame", "data": frame_data}))
        except Exception:
            dead.add(ws)
    for ws in dead:
        viewer_subscribers.discard(ws)


async def broadcast_video_ready(video_url: str):
    """Broadcast notification that a replay video is ready."""
    dead = set()
    for ws in viewer_subscribers:
        try:
            await ws.send_text(json.dumps({"type": "video_ready", "video_url": video_url}))
        except Exception:
            dead.add(ws)
    for ws in dead:
        viewer_subscribers.discard(ws)


# IMPORTANT: /ws/sim MUST be defined BEFORE /ws/{order_id} to avoid route conflict
@router.websocket("/ws/sim")
async def sim_websocket(websocket: WebSocket):
    """Channel for sim clients waiting for orders. Backend sends new_order when user submits a task."""
    await websocket.accept()
    sim_subscribers.add(websocket)
    logger.info(f"Sim client connected. Total sim_subscribers: {len(sim_subscribers)}")
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        sim_subscribers.discard(websocket)
        logger.info(f"Sim client disconnected. Total sim_subscribers: {len(sim_subscribers)}")


@router.websocket("/ws/viewer")
async def viewer_websocket(websocket: WebSocket):
    """Channel for frontend clients to receive simulation frames."""
    await websocket.accept()
    viewer_subscribers.add(websocket)
    logger.info(f"Viewer client connected. Total viewer_subscribers: {len(viewer_subscribers)}")
    # Send connection status
    await websocket.send_text(json.dumps({"type": "connected", "message": "Viewer connected"}))
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        viewer_subscribers.discard(websocket)
        logger.info(f"Viewer client disconnected. Total viewer_subscribers: {len(viewer_subscribers)}")


@router.websocket("/ws/{order_id}")
async def order_websocket(websocket: WebSocket, order_id: str):
    await websocket.accept()
    connections.setdefault(order_id, set()).add(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        connections.get(order_id, set()).discard(websocket)
