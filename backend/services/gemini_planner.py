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
                        "enum": ["move_to_pick", "pick", "move_to_delivery", "place"],
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

SYSTEM_PROMPT = """You are a robot task planner for a cleanup room. The robot picks up objects and places them in a bin.

Given a cleanup list (objects and quantities to pick up), generate a complete execution plan as a JSON array of steps.

Each step must be one of:
- "move_to_pick": Move robot arm to item position
- "pick": Grasp/pick the item
- "move_to_delivery": Move robot arm to the bin
- "place": Release/place the item in the bin

Rules:
- For each item, you must execute: move_to_pick → pick → move_to_delivery → place (in that order)
- Repeat this 4-step sequence for each unit of each item
- Example: 2 apples + 1 water = [move_to_pick(apple), pick(apple), move_to_delivery(apple), place(apple), move_to_pick(apple), pick(apple), move_to_delivery(apple), place(apple), move_to_pick(water), pick(water), move_to_delivery(water), place(water)]

Generate a complete plan with all steps. Include a brief "reasoning" field explaining your strategy."""


def _model_for_plan() -> str:
    """Use robotics model when set, else default Gemini model."""
    return getattr(settings, "gemini_robotics_model", None) or settings.gemini_model


async def plan_pick_place_sequence(pick_list: List[PickListItem]) -> dict:
    """
    Use Gemini to generate the full pick-place plan upfront.
    Returns: {"steps": [{"step": str, "item_id": str, "message": str}, ...], "reasoning": str}
    """
    pick_list_str = json.dumps([{"item_id": p.item_id, "quantity": p.quantity} for p in pick_list])
    prompt = f"Order to fulfill:\n{pick_list_str}\n\nGenerate the complete execution plan."
    model = _model_for_plan()

    try:
        response = await client.aio.models.generate_content(
            model=model,
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
        if model != settings.gemini_model:
            try:
                response = await client.aio.models.generate_content(
                    model=settings.gemini_model,
                    contents=prompt,
                    config=types.GenerateContentConfig(
                        system_instruction=SYSTEM_PROMPT,
                        response_mime_type="application/json",
                        response_schema=PLAN_SCHEMA,
                        temperature=0.2,
                    ),
                )
                return json.loads(response.text)
            except Exception:
                pass
        # Fallback: generate plan deterministically
        steps = []
        for item in pick_list:
            for _ in range(item.quantity):
                steps.extend([
                    {"step": "move_to_pick", "item_id": item.item_id, "message": f"Moving to {item.item_id}"},
                    {"step": "pick", "item_id": item.item_id, "message": f"Picking {item.item_id}"},
                    {"step": "move_to_delivery", "item_id": item.item_id, "message": f"Taking {item.item_id} to bin"},
                    {"step": "place", "item_id": item.item_id, "message": f"Placed {item.item_id} in bin"},
                ])
        return {"steps": steps, "reasoning": "Fallback deterministic plan"}
