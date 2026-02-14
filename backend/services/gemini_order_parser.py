"""Parse natural language cleanup requests into structured pick lists using Gemini."""
import json
import logging
from typing import List

from google import genai
from google.genai import types

from config import settings
from schemas import PickListItem

logger = logging.getLogger(__name__)

client = genai.Client(api_key=settings.gemini_api_key)

# Room objects the robot can pick up and put in the bin — item_id must match simulation objects
ROOM_OBJECTS = {
    "banana": "Banana",
    "duck": "Rubber duck toy",
    "phone": "Phone",
    "elephant": "Elephant toy",
    "eyeglasses": "Eyeglasses",
    "flute": "Flute",
    "gamecontroller": "Game controller",
    "headphones": "Headphones",
    "mouse": "Computer mouse",
    "piggybank": "Piggy bank",
    "pyramidlarge": "Pyramid toy",
    "stanfordbunny": "Bunny toy",
    "train": "Toy train",
    "watch": "Watch",
    "airplane": "Toy airplane",
    "alarmclock": "Alarm clock",
    "camera": "Camera",
}
ROOM_OBJECT_IDS = list(ROOM_OBJECTS.keys())

SYSTEM_PROMPT = f"""You are a cleanup-room task parser. The user says what to tidy or pick up; you output a JSON list of objects to pick up and put in the bin.

Available objects (use these exact item_id values): {json.dumps(ROOM_OBJECT_IDS)}

Output a JSON array of objects, each with:
- "item_id": one of the available object IDs (string)
- "quantity": positive integer (how many of that object to pick up)

Examples:
- "Pick up the banana and the duck" -> [{{"item_id": "banana", "quantity": 1}}, {{"item_id": "duck", "quantity": 1}}]
- "Clean up the toys" -> [{{"item_id": "duck", "quantity": 1}}, {{"item_id": "elephant", "quantity": 1}}, {{"item_id": "train", "quantity": 1}}, {{"item_id": "airplane", "quantity": 1}}]
- "Put the phone and headphones in the bin" -> [{{"item_id": "phone", "quantity": 1}}, {{"item_id": "headphones", "quantity": 1}}]
- "Tidy up all the clutter" -> [{{"item_id": "banana", "quantity": 1}}, {{"item_id": "duck", "quantity": 1}}, {{"item_id": "phone", "quantity": 1}}, {{"item_id": "elephant", "quantity": 1}}, {{"item_id": "eyeglasses", "quantity": 1}}, {{"item_id": "flute", "quantity": 1}}, {{"item_id": "gamecontroller", "quantity": 1}}, {{"item_id": "headphones", "quantity": 1}}, {{"item_id": "mouse", "quantity": 1}}, {{"item_id": "piggybank", "quantity": 1}}, {{"item_id": "pyramidlarge", "quantity": 1}}, {{"item_id": "stanfordbunny", "quantity": 1}}, {{"item_id": "train", "quantity": 1}}, {{"item_id": "watch", "quantity": 1}}, {{"item_id": "airplane", "quantity": 1}}, {{"item_id": "alarmclock", "quantity": 1}}, {{"item_id": "camera", "quantity": 1}}]

Rules:
- Only include objects from the available list. Map common words to the list (e.g. "rubber ducky" -> duck, "toy bunny" -> stanfordbunny, "controller" -> gamecontroller).
- Respond ONLY with a valid JSON array. No markdown, no extra text."""

PICK_LIST_SCHEMA = {
    "type": "array",
    "items": {
        "type": "object",
        "properties": {
            "item_id": {"type": "string"},
            "quantity": {"type": "integer"},
        },
        "required": ["item_id", "quantity"],
    },
}


def _model_for_parse() -> str:
    """Use robotics model when set, else default Gemini model."""
    return getattr(settings, "gemini_robotics_model", None) or settings.gemini_model


async def parse_order(natural_language_input: str) -> List[PickListItem]:
    """Use Gemini to parse natural language into a validated pick list."""
    model = _model_for_parse()
    try:
        response = await client.aio.models.generate_content(
            model=model,
            contents=f"User request: {natural_language_input}",
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                response_mime_type="application/json",
                response_schema=PICK_LIST_SCHEMA,
                temperature=0.2,
            ),
        )
        raw = json.loads(response.text)
        if not isinstance(raw, list):
            raw = [raw]
        # Validate and filter to known room objects
        result = []
        for obj in raw:
            item_id = (obj.get("item_id") or "").strip().lower()
            qty = max(1, int(obj.get("quantity") or 1))
            if item_id in ROOM_OBJECT_IDS:
                result.append(PickListItem(item_id=item_id, quantity=qty))
        if not result:
            result = [PickListItem(item_id="duck", quantity=1)]
        logger.info("Gemini cleanup parse: %s -> %s", natural_language_input, result)
        return result
    except Exception as e:
        logger.error("Gemini order parser failed: %s", e)
        if model != settings.gemini_model:
            try:
                response = await client.aio.models.generate_content(
                    model=settings.gemini_model,
                    contents=f"User request: {natural_language_input}",
                    config=types.GenerateContentConfig(
                        system_instruction=SYSTEM_PROMPT,
                        response_mime_type="application/json",
                        response_schema=PICK_LIST_SCHEMA,
                        temperature=0.2,
                    ),
                )
                raw = json.loads(response.text)
                if not isinstance(raw, list):
                    raw = [raw]
                result = []
                for obj in raw:
                    item_id = (obj.get("item_id") or "").strip().lower()
                    qty = max(1, int(obj.get("quantity") or 1))
                    if item_id in ROOM_OBJECT_IDS:
                        result.append(PickListItem(item_id=item_id, quantity=qty))
                if result:
                    return result
            except Exception:
                pass
        return [PickListItem(item_id="duck", quantity=1)]
