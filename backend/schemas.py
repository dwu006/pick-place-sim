from datetime import datetime
from typing import Dict, List, Optional

from pydantic import BaseModel


class PourSpec(BaseModel):
    pattern: str
    pour_height: float
    oscillation_freq: float
    flow_rate: float
    pull_through: bool


class JobCreateRequest(BaseModel):
    natural_language_input: str
    pattern_preset: Optional[str] = None


class JobResponse(BaseModel):
    id: str
    pattern_name: str
    natural_language_input: str
    pour_spec: Optional[PourSpec] = None
    status: str
    video_url: Optional[str] = None
    score: Optional[float] = None
    feedback: Optional[str] = None
    breakdown: Optional[Dict] = None
    error_message: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class PatternPreset(BaseModel):
    id: str
    name: str
    description: str


class EvaluationResult(BaseModel):
    score: float
    feedback: str
    breakdown: Dict
