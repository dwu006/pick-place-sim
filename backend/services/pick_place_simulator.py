"""Simulated pick-and-place: hardcoded IK-style steps, no learning. Broadcasts steps over WebSocket."""
import asyncio
import logging
from typing import List

from schemas import PickListItem
from routers.websocket import broadcast

logger = logging.getLogger(__name__)


async def run_pick_place(order_id: str, pick_list: List[PickListItem]) -> None:
    """
    Run a simulated pick-and-place sequence. For each item we emit
    steps (move to shelf -> pick -> move to table -> place) with short delays.
    No real IK; this is demonstrative for the hackathon.
    """
    for idx, line in enumerate(pick_list):
        for q in range(line.quantity):
            # Step 1: Move to shelf
            await broadcast(
                order_id,
                {
                    "type": "robot_step",
                    "step": "move_to_shelf",
                    "item_id": line.item_id,
                    "quantity_index": q + 1,
                    "message": f"Moving to shelf for {line.item_id}",
                },
            )
            await asyncio.sleep(0.8)

            # Step 2: Pick
            await broadcast(
                order_id,
                {
                    "type": "robot_step",
                    "step": "pick",
                    "item_id": line.item_id,
                    "message": f"Picking {line.item_id}",
                },
            )
            await asyncio.sleep(0.6)

            # Step 3: Move to delivery area
            await broadcast(
                order_id,
                {
                    "type": "robot_step",
                    "step": "move_to_delivery",
                    "item_id": line.item_id,
                    "message": f"Placing {line.item_id} in front of you",
                },
            )
            await asyncio.sleep(0.7)

            # Step 4: Place
            await broadcast(
                order_id,
                {
                    "type": "robot_step",
                    "step": "place",
                    "item_id": line.item_id,
                    "message": f"Placed {line.item_id}",
                },
            )
            await asyncio.sleep(0.4)

    await broadcast(order_id, {"type": "robot_step", "step": "done", "message": "Order ready."})
