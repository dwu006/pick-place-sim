import asyncio
import logging
from datetime import datetime, timezone

from sqlalchemy import select

from database import async_session
from models import Job
from routers.websocket import broadcast
from services.gemini_evaluator import evaluate_result
from services.gemini_planner import plan_pour
from services.fluid_simulator import run_simulation

logger = logging.getLogger(__name__)


async def _process_job(job_id: str):
    """Process a single job through the full pipeline."""
    async with async_session() as db:
        job = await db.get(Job, job_id)
        if not job:
            return

        try:
            # Stage 1: Planning
            job.status = "planning"
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {"type": "status_update", "job_id": job_id, "status": "planning"})

            pour_spec = await plan_pour(job.natural_language_input, job.pattern_name)
            job.pour_spec = pour_spec.model_dump()
            job.pattern_name = pour_spec.pattern
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {
                "type": "pour_spec_ready",
                "job_id": job_id,
                "pour_spec": pour_spec.model_dump(),
            })

            # Stage 2: Simulation
            job.status = "simulating"
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {"type": "status_update", "job_id": job_id, "status": "simulating"})

            video_url = await run_simulation(pour_spec, job_id)
            job.video_url = video_url
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {
                "type": "simulation_complete",
                "job_id": job_id,
                "video_url": video_url,
            })

            # Stage 3: Evaluation
            job.status = "evaluating"
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {"type": "status_update", "job_id": job_id, "status": "evaluating"})

            eval_result = await evaluate_result(video_url, pour_spec)
            job.score = eval_result.score
            job.feedback = eval_result.feedback
            job.breakdown = eval_result.breakdown
            job.status = "completed"
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {
                "type": "evaluation_complete",
                "job_id": job_id,
                "score": eval_result.score,
                "feedback": eval_result.feedback,
                "breakdown": eval_result.breakdown,
            })

        except Exception as e:
            logger.exception(f"Job {job_id} failed")
            job.status = "failed"
            job.error_message = str(e)
            job.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await broadcast(job_id, {
                "type": "error",
                "job_id": job_id,
                "error": str(e),
            })


async def start_worker():
    """Poll for queued jobs and process them."""
    logger.info("Worker started")
    while True:
        try:
            async with async_session() as db:
                result = await db.execute(
                    select(Job).where(Job.status == "queued").order_by(Job.created_at).limit(1)
                )
                job = result.scalar_one_or_none()

            if job:
                await _process_job(job.id)
            else:
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            logger.info("Worker shutting down")
            break
        except Exception:
            logger.exception("Worker error")
            await asyncio.sleep(2.0)
