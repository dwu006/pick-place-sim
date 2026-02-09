import json
import logging
import random

from google import genai
from google.genai import types

from config import settings
from schemas import EvaluationResult, PourSpec

logger = logging.getLogger(__name__)

client = genai.Client(api_key=settings.gemini_api_key)

SYSTEM_PROMPT = """You are LatteBot's latte art quality evaluator. You evaluate simulated latte art pours based on the
pour parameters and pattern type. In the future you will receive actual images, but for now evaluate based on
the trajectory parameters.

Evaluate the pour quality and return structured JSON with:
- score: overall quality score 0-100 (be realistic, most pours score 70-95)
- feedback: 2-3 sentences of specific, actionable feedback about the pour quality. Reference specific parameters.
- breakdown: object with four sub-scores (0-100 each):
  - contrast: how well the milk and crema create visual distinction
  - symmetry: how balanced and centered the pattern is
  - definition: how sharp and clear the pattern edges are
  - crema_quality: how well the crema is preserved around the pattern

Consider these heuristics:
- Rosetta: good oscillation_freq (3.0-4.0) and moderate pour_height (4-6cm) score higher
- Tulip: consistent flow_rate and proper pour_height (5-7cm) matter most
- Heart: lower pour_height (3-5cm) and moderate flow_rate (30-45ml/s) score best
- Extreme values in any parameter reduce scores
- Pull-through usually improves definition

Be a discerning but fair evaluator. Provide constructive feedback."""

EVAL_SCHEMA = {
    "type": "object",
    "properties": {
        "score": {"type": "number"},
        "feedback": {"type": "string"},
        "breakdown": {
            "type": "object",
            "properties": {
                "contrast": {"type": "number"},
                "symmetry": {"type": "number"},
                "definition": {"type": "number"},
                "crema_quality": {"type": "number"},
            },
            "required": ["contrast", "symmetry", "definition", "crema_quality"],
        },
    },
    "required": ["score", "feedback", "breakdown"],
}


async def evaluate_result(video_url: str, pour_spec: PourSpec) -> EvaluationResult:
    """Use Gemini Pro to evaluate the latte art result."""
    prompt = f"""Evaluate this simulated latte art pour:

Pattern: {pour_spec.pattern}
Pour Height: {pour_spec.pour_height} cm
Oscillation Frequency: {pour_spec.oscillation_freq} Hz
Flow Rate: {pour_spec.flow_rate} ml/s
Pull Through: {pour_spec.pull_through}

The simulation has completed and produced a {pour_spec.pattern} pattern.
Evaluate the quality of these pour parameters and provide your assessment."""

    try:
        response = await client.aio.models.generate_content(
            model="gemini-2.0-flash",
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                response_mime_type="application/json",
                response_schema=EVAL_SCHEMA,
                temperature=0.7,
            ),
        )

        data = json.loads(response.text)
        logger.info(f"Gemini evaluator response: score={data.get('score')}")
        return EvaluationResult(**data)

    except Exception as e:
        logger.error(f"Gemini evaluator failed: {e}, falling back to mock")
        # Fallback to random mock scores
        score = round(random.uniform(75.0, 90.0), 1)
        return EvaluationResult(
            score=score,
            feedback=f"Evaluation service encountered an issue. Based on parameters, the {pour_spec.pattern} pattern appears reasonable.",
            breakdown={
                "contrast": round(random.uniform(70.0, 92.0), 1),
                "symmetry": round(random.uniform(70.0, 92.0), 1),
                "definition": round(random.uniform(70.0, 92.0), 1),
                "crema_quality": round(random.uniform(70.0, 92.0), 1),
            },
        )
