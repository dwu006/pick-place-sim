import asyncio
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from config import settings
from database import engine
from models import Base
from routers import jobs, patterns, websocket
from worker import start_worker

logging.basicConfig(level=logging.INFO)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Create tables
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    # Start background worker
    worker_task = asyncio.create_task(start_worker())
    yield
    worker_task.cancel()
    try:
        await worker_task
    except asyncio.CancelledError:
        pass


app = FastAPI(title="LatteBot API", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory="static"), name="static")

app.include_router(jobs.router, prefix="/api", tags=["jobs"])
app.include_router(patterns.router, prefix="/api", tags=["patterns"])
app.include_router(websocket.router, tags=["websocket"])


@app.get("/")
def root():
    return {"message": "LatteBot API v1.0", "status": "operational"}


@app.get("/health")
def health():
    return {"status": "healthy"}
