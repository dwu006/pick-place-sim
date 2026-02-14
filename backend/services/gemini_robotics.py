"""
Thin wrapper for Gemini Robotics-ER 1.5 (vision-language robotics model).
Accepts prompt and optional image bytes; returns parsed response (e.g. object points/labels).
Initially text-only; add image input when wrist-camera rendering is available.
"""
import json
import logging
from typing import Any, Optional

from google import genai
from google.genai import types

from config import settings

logger = logging.getLogger(__name__)

client = genai.Client(api_key=settings.gemini_api_key)


async def generate_content_robotics(
    prompt: str,
    image_bytes: Optional[bytes] = None,
    mime_type: str = "image/png",
    **kwargs: Any,
) -> str:
    """
    Call Gemini Robotics-ER 1.5 with prompt and optional image.
    Returns response text. Use for object points, labels, or function-call sequences.
    """
    model = getattr(settings, "gemini_robotics_model", None) or getattr(
        settings, "gemini_model", "gemini-robotics-er-1.5-preview"
    )
    parts = [prompt]
    if image_bytes:
        parts.insert(
            0,
            types.Part.from_bytes(data=image_bytes, mime_type=mime_type),
        )
    config = types.GenerateContentConfig(
        temperature=kwargs.get("temperature", 0.5),
        **{k: v for k, v in kwargs.items() if k != "temperature"},
    )
    response = await client.aio.models.generate_content(
        model=model,
        contents=parts,
        config=config,
    )
    return response.text


async def generate_robotics_json(
    prompt: str,
    image_bytes: Optional[bytes] = None,
    **kwargs: Any,
) -> Any:
    """Call robotics model and parse response as JSON."""
    text = await generate_content_robotics(prompt, image_bytes=image_bytes, **kwargs)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        logger.warning("Robotics model response was not valid JSON: %s", text[:200])
        return None
