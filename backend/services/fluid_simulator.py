import asyncio

from schemas import PourSpec


async def run_simulation(pour_spec: PourSpec, job_id: str) -> str:
    """Mock FluidLab simulation. Returns path to video file."""
    await asyncio.sleep(4.0)  # simulate GPU compute time

    # Map to pre-made video (placeholder — replace with real sim later)
    return f"/static/videos/{pour_spec.pattern}.mp4"
