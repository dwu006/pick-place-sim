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

# Room objects the robot can pick up and put in the bin — item_id must match these
ROOM_OBJECTS = {
    "red_block": "Red block",
    "blue_block": "Blue block",
    "green_block": "Green block",
    "cup": "Cup",
    "bottle": "Bottle",
    "toy": "Toy",
    "book": "Book",
    "box": "Box",
}
ROOM_OBJECT_IDS = list(ROOM_OBJECTS.keys())

SYSTEM_PROMPT = f"""You are a cleanup-room task parser. The user says what to tidy or pick up; you output a JSON list of objects to pick up and put in the bin.

Available objects (use these exact item_id values): {json.dumps(ROOM_OBJECT_IDS)}

Output a JSON array of objects, each with:
- "item_id": one of the available object IDs (string)
- "quantity": positive integer (how many of that object to pick up)

Examples:
- "Pick up the red block and the cup" -> [{{"item_id": "red_block", "quantity": 1}}, {{"item_id": "cup", "quantity": 1}}]
- "Tidy the blue block and two bottles" -> [{{"item_id": "blue_block", "quantity": 1}}, {{"item_id": "bottle", "quantity": 2}}]
- "Clean up the toys and the book" -> [{{"item_id": "toy", "quantity": 1}}, {{"item_id": "book", "quantity": 1}}]
- "Put the green block in the bin" -> [{{"item_id": "green_block", "quantity": 1}}]

Rules:
- Only include objects from the available list. Map common words to the list (e.g. "red cube" -> red_block, "mug" -> cup, "bottle" -> bottle).
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


async def parse_order(natural_language_input: str) -> List[PickListItem]:
    """Use Gemini to parse natural language into a validated pick list."""
    try:
        response = await client.aio.models.generate_content(
            model="gemini-2.0-flash",
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
            result = [PickListItem(item_id="red_block", quantity=1)]
        logger.info("Gemini cleanup parse: %s -> %s", natural_language_input, result)
        return result
    except Exception as e:
        logger.error("Gemini order parser failed: %s", e)
        return [PickListItem(item_id="red_block", quantity=1)]
