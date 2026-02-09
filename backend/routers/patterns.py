from typing import List

from fastapi import APIRouter

from schemas import PatternPreset

router = APIRouter()

PRESETS = [
    PatternPreset(
        id="rosetta",
        name="Rosetta",
        description="Classic layered leaf pattern with high contrast",
    ),
    PatternPreset(
        id="tulip",
        name="Tulip",
        description="Stacked tulip petals, requires precise pour control",
    ),
    PatternPreset(
        id="heart",
        name="Heart",
        description="Simple heart shape, beginner-friendly",
    ),
]


@router.get("/patterns", response_model=List[PatternPreset])
async def list_patterns():
    return PRESETS
