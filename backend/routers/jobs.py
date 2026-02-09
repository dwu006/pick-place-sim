from typing import List

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from database import get_db
from models import Job
from schemas import JobCreateRequest, JobResponse

router = APIRouter()


@router.post("/jobs", response_model=JobResponse, status_code=201)
async def create_job(req: JobCreateRequest, db: AsyncSession = Depends(get_db)):
    pattern_name = req.pattern_preset or "custom"

    job = Job(
        pattern_name=pattern_name,
        natural_language_input=req.natural_language_input,
        status="queued",
    )
    db.add(job)
    await db.commit()
    await db.refresh(job)
    return job


@router.get("/jobs", response_model=List[JobResponse])
async def list_jobs(db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(Job).order_by(Job.created_at.desc()).limit(20))
    return result.scalars().all()


@router.get("/jobs/{job_id}", response_model=JobResponse)
async def get_job(job_id: str, db: AsyncSession = Depends(get_db)):
    job = await db.get(Job, job_id)
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    return job
