"""
Gemini-driven pick-and-place: plan-first approach (better than function calling).
Gemini generates the full plan upfront, then we execute it step-by-step.
Shows AI reasoning/planning while keeping execution fast and deterministic.
"""
import asyncio
import logging
from typing import List

from schemas import PickListItem
from services.gemini_planner import plan_pick_place_sequence
from routers.websocket import broadcast

logger = logging.getLogger(__name__)


async def run_pick_place_with_gemini(
    order_id: str,
    pick_list: List[PickListItem],
) -> None:
    """
    Gemini generates the full plan upfront (single API call), then we execute it.
    This is faster and cheaper than function calling, while still showing Gemini's planning.
    """
    # Get full plan from Gemini (single API call)
    plan = await plan_pick_place_sequence(pick_list)
    steps = plan.get("steps", [])
    reasoning = plan.get("reasoning", "")

    logger.info("Executing Gemini plan: %d steps. Reasoning: %s", len(steps), reasoning[:100])

    # Broadcast reasoning (optional - shows Gemini's thinking)
    if reasoning:
        try:
            await broadcast(
                order_id,
                {
                    "type": "robot_step",
                    "step": "planning",
                    "message": f"Plan: {reasoning}",
                },
            )
        except Exception:
            pass

    # Execute plan step-by-step
    delays = {
        "move_to_shelf": 0.8,
        "pick": 0.6,
        "move_to_delivery": 0.7,
        "place": 0.4,
    }

    for step_data in steps:
        step_name = step_data.get("step", "")
        item_id = step_data.get("item_id", "")
        message = step_data.get("message", f"{step_name} {item_id}")

        await broadcast(
            order_id,
            {
                "type": "robot_step",
                "step": step_name,
                "item_id": item_id,
                "message": message,
            },
        )
        await asyncio.sleep(delays.get(step_name, 0.5))

    await broadcast(order_id, {"type": "robot_step", "step": "done", "message": "Order ready."})
