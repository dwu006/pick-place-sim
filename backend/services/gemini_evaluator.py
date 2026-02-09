import asyncio
import random

from schemas import EvaluationResult, PourSpec

FEEDBACK_TEMPLATES: dict[str, list[str]] = {
    "rosetta": [
        "Good leaf definition and symmetry. The rosetta layers are well-separated with clear contrast against the crema.",
        "Excellent rosetta structure. Consider slightly reducing flow rate mid-pour for tighter leaf spacing.",
        "Strong rosetta pattern. The pull-through creates a clean central spine. Minor asymmetry on the left side.",
    ],
    "tulip": [
        "Well-stacked tulip petals with good separation. The base is slightly wide — try a higher pour to tighten it.",
        "Clean tulip formation. Each layer is distinct. Crema integration could be improved with a slower initial pour.",
        "Solid tulip pattern. The top petal is well-defined. Consider more pause between layers for sharper edges.",
    ],
    "heart": [
        "Clean heart shape with smooth curves. Great contrast between milk and crema.",
        "Well-formed heart. The symmetry is excellent. Slightly reduce pour height for a more defined point at the base.",
        "Nice heart pattern. The pull-through creates a clean finish. Milk density is even throughout.",
    ],
}

SCORE_RANGES: dict[str, tuple[float, float]] = {
    "rosetta": (82.0, 94.0),
    "tulip": (78.0, 90.0),
    "heart": (85.0, 96.0),
}


async def evaluate_result(video_url: str, pour_spec: PourSpec) -> EvaluationResult:
    """Mock Gemini Pro multimodal evaluation."""
    await asyncio.sleep(2.0)  # simulate API latency

    pattern = pour_spec.pattern
    lo, hi = SCORE_RANGES.get(pattern, (75.0, 88.0))
    score = round(random.uniform(lo, hi), 1)

    feedback_options = FEEDBACK_TEMPLATES.get(pattern, FEEDBACK_TEMPLATES["heart"])
    feedback = random.choice(feedback_options)

    breakdown = {
        "contrast": round(random.uniform(lo, hi), 1),
        "symmetry": round(random.uniform(lo, hi), 1),
        "definition": round(random.uniform(lo, hi), 1),
        "crema_quality": round(random.uniform(lo, hi), 1),
    }

    return EvaluationResult(score=score, feedback=feedback, breakdown=breakdown)
