import asyncio
import re
from typing import Dict, Optional

from schemas import PourSpec

PATTERN_SPECS: Dict[str, PourSpec] = {
    "rosetta": PourSpec(
        pattern="rosetta",
        pour_height=5.0,
        oscillation_freq=3.5,
        flow_rate=45.0,
        pull_through=True,
    ),
    "tulip": PourSpec(
        pattern="tulip",
        pour_height=6.0,
        oscillation_freq=0.0,
        flow_rate=50.0,
        pull_through=True,
    ),
    "heart": PourSpec(
        pattern="heart",
        pour_height=4.5,
        oscillation_freq=0.0,
        flow_rate=40.0,
        pull_through=True,
    ),
}

PATTERN_KEYWORDS = ["rosetta", "tulip", "heart"]


def _detect_pattern(text: str) -> str:
    lower = text.lower()
    for kw in PATTERN_KEYWORDS:
        if re.search(rf"\b{kw}\b", lower):
            return kw
    return "heart"  # default fallback


async def plan_pour(nl_input: str, preset: Optional[str] = None) -> PourSpec:
    """Mock Gemini Flash: parse natural language into pour specification."""
    await asyncio.sleep(1.0)  # simulate API latency

    pattern = preset if preset and preset in PATTERN_SPECS else _detect_pattern(nl_input)
    return PATTERN_SPECS[pattern]
