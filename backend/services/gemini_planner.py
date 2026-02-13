"""
Gemini generates the full pick-place plan upfront (structured output).
Shows AI reasoning/planning while keeping execution fast and deterministic.
"""
import json
import logging
from typing import List

from google import genai
from google.genai import types

from config import settings
from schemas import PickListItem

logger = logging.getLogger(__name__)

client = genai.Client(api_key=settings.gemini_api_key)

PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "steps": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "step": {
                        "type": "string",
                        "enum": ["move_to_shelf", "pick", "move_to_delivery", "place"],
                    },
                    "item_id": {"type": "string"},
                    "message": {"type": "string"},
                },
                "required": ["step", "item_id", "message"],
            },
        },
        "reasoning": {
            "type": "string",
            "description": "Brief explanation of the plan strategy",
        },
    },
    "required": ["steps", "reasoning"],
}

SYSTEM_PROMPT = """You are a robot task planner for a mini store pick-and-place system.

Given an order (list of items and quantities), generate a complete execution plan as a JSON array of steps.

Each step must be one of:
- "move_to_shelf": Move robot arm to shelf position for an item
- "pick": Grasp/pick the item from shelf
- "move_to_delivery": Move robot arm to delivery area in front of user
- "place": Release/place the item in delivery area

Rules:
- For each item, you must execute: move_to_shelf → pick → move_to_delivery → place (in that order)
- Repeat this 4-step sequence for each unit of each item
- Example: 2 apples + 1 water = [move_to_shelf(apple), pick(apple), move_to_delivery(apple), place(apple), move_to_shelf(apple), pick(apple), move_to_delivery(apple), place(apple), move_to_shelf(water), pick(water), move_to_delivery(water), place(water)]

Generate a complete plan with all steps. Include a brief "reasoning" field explaining your strategy."""


async def plan_pick_place_sequence(pick_list: List[PickListItem]) -> dict:
    """
    Use Gemini to generate the full pick-place plan upfront.
    Returns: {"steps": [{"step": str, "item_id": str, "message": str}, ...], "reasoning": str}
    """
    pick_list_str = json.dumps([{"item_id": p.item_id, "quantity": p.quantity} for p in pick_list])
    prompt = f"Order to fulfill:\n{pick_list_str}\n\nGenerate the complete execution plan."

    try:
        response = await client.aio.models.generate_content(
            model="gemini-2.0-flash",
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                response_mime_type="application/json",
                response_schema=PLAN_SCHEMA,
                temperature=0.2,
            ),
        )
        plan = json.loads(response.text)
        logger.info("Gemini plan: %d steps, reasoning: %s", len(plan.get("steps", [])), plan.get("reasoning", "")[:100])
        return plan
    except Exception as e:
        logger.error("Gemini planner failed: %s", e)
        # Fallback: generate plan deterministically
        steps = []
        for item in pick_list:
            for _ in range(item.quantity):
                steps.extend([
                    {"step": "move_to_shelf", "item_id": item.item_id, "message": f"Moving to shelf for {item.item_id}"},
                    {"step": "pick", "item_id": item.item_id, "message": f"Picking {item.item_id}"},
                    {"step": "move_to_delivery", "item_id": item.item_id, "message": f"Placing {item.item_id} in front of you"},
                    {"step": "place", "item_id": item.item_id, "message": f"Placed {item.item_id}"},
                ])
        return {"steps": steps, "reasoning": "Fallback deterministic plan"}
