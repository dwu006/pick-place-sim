import json
import logging
from typing import Optional

from google import genai
from google.genai import types

from config import settings
from schemas import PourSpec

logger = logging.getLogger(__name__)

client = genai.Client(api_key=settings.gemini_api_key)

SYSTEM_PROMPT = """You are LatteBot's pour planning AI. Given a natural language description of a latte art pattern,
output the optimal pour parameters as structured JSON.

Parameters:
- pattern: one of "rosetta", "tulip", "heart", or a custom name (string)
- pour_height: height of pitcher above cup in cm (2.0-10.0). Lower = more detail, higher = wider spread.
- oscillation_freq: side-to-side frequency in Hz (0.0-6.0). 0 for no oscillation (heart/tulip). Higher for rosetta layers.
- flow_rate: milk flow rate in ml/s (20.0-80.0). Controls density and spread.
- pull_through: whether to pull through the pattern at the end (boolean). Usually true.

Guidelines:
- Rosetta: oscillation_freq 2.5-4.5, pour_height 4-6, flow_rate 35-55, pull_through true
- Tulip: oscillation_freq 0, pour_height 5-7, flow_rate 40-60, pull_through true (push-back technique)
- Heart: oscillation_freq 0, pour_height 3-5, flow_rate 30-50, pull_through true
- Adjust based on user descriptors like "tight", "wide", "high contrast", "delicate", etc.

Respond ONLY with valid JSON matching the schema. No extra text."""

POUR_SPEC_SCHEMA = {
    "type": "object",
    "properties": {
        "pattern": {"type": "string"},
        "pour_height": {"type": "number"},
        "oscillation_freq": {"type": "number"},
        "flow_rate": {"type": "number"},
        "pull_through": {"type": "boolean"},
    },
    "required": ["pattern", "pour_height", "oscillation_freq", "flow_rate", "pull_through"],
}


async def plan_pour(nl_input: str, preset: Optional[str] = None) -> PourSpec:
    """Use Gemini Flash to parse natural language into pour specification."""
    prompt = f"User request: {nl_input}"
    if preset:
        prompt += f"\nPreset pattern selected: {preset}"

    try:
        response = await client.aio.models.generate_content(
            model="gemini-2.0-flash",
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                response_mime_type="application/json",
                response_schema=POUR_SPEC_SCHEMA,
                temperature=0.3,
            ),
        )

        data = json.loads(response.text)
        logger.info(f"Gemini planner response: {data}")
        return PourSpec(**data)

    except Exception as e:
        logger.error(f"Gemini planner failed: {e}, falling back to defaults")
        # Fallback to sensible defaults
        pattern = preset or "heart"
        defaults = {
            "rosetta": PourSpec(pattern="rosetta", pour_height=5.0, oscillation_freq=3.5, flow_rate=45.0, pull_through=True),
            "tulip": PourSpec(pattern="tulip", pour_height=6.0, oscillation_freq=0.0, flow_rate=50.0, pull_through=True),
            "heart": PourSpec(pattern="heart", pour_height=4.5, oscillation_freq=0.0, flow_rate=40.0, pull_through=True),
        }
        return defaults.get(pattern, defaults["heart"])
