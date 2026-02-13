"""Parse natural language store orders into structured pick lists using Gemini."""
import json
import logging
from typing import List

from google import genai
from google.genai import types

from config import settings
from schemas import PickListItem

logger = logging.getLogger(__name__)

client = genai.Client(api_key=settings.gemini_api_key)

# Fixed store inventory — item_id must match these
STORE_INVENTORY = {
    "apple": "Apple",
    "banana": "Banana",
    "water": "Water bottle",
    "chips": "Chips",
    "soda": "Soda",
    "cookie": "Cookie",
    "sandwich": "Sandwich",
    "coffee": "Coffee",
}
INVENTORY_IDS = list(STORE_INVENTORY.keys())

SYSTEM_PROMPT = f"""You are a mini store order parser. The user says what they want; you output a JSON list of items to pick.

Available items (use these exact item_id values): {json.dumps(INVENTORY_IDS)}

Output a JSON array of objects, each with:
- "item_id": one of the available item IDs (string)
- "quantity": positive integer

Examples:
- "I want two apples and a water" -> [{{"item_id": "apple", "quantity": 2}}, {{"item_id": "water", "quantity": 1}}]
- "Give me a coffee and 3 cookies" -> [{{"item_id": "coffee", "quantity": 1}}, {{"item_id": "cookie", "quantity": 3}}]
- "Just a sandwich" -> [{{"item_id": "sandwich", "quantity": 1}}]

Rules:
- Only include items from the available list. If the user asks for something not in the list, skip it or map to the closest match (e.g. "drink" -> water or soda).
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
        # Validate and filter to known inventory
        result = []
        for obj in raw:
            item_id = (obj.get("item_id") or "").strip().lower()
            qty = max(1, int(obj.get("quantity") or 1))
            if item_id in INVENTORY_IDS:
                result.append(PickListItem(item_id=item_id, quantity=qty))
        if not result:
            result = [PickListItem(item_id="apple", quantity=1)]
        logger.info("Gemini order parse: %s -> %s", natural_language_input, result)
        return result
    except Exception as e:
        logger.error("Gemini order parser failed: %s", e)
        return [PickListItem(item_id="apple", quantity=1)]
