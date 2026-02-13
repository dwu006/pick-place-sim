import asyncio
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from config import settings
from database import engine
from models import Base
from routers import orders, websocket
from worker import start_worker

logging.basicConfig(level=logging.INFO)


@asynccontextmanager
async def lifespan(app: FastAPI):
    if settings.use_database:
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
    worker_task = asyncio.create_task(start_worker())
    yield
    worker_task.cancel()
    try:
        await worker_task
    except asyncio.CancelledError:
        pass


app = FastAPI(title="Mini Store Pick & Place API", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory="static"), name="static")

app.include_router(orders.router, prefix="/api", tags=["orders"])
app.include_router(websocket.router, tags=["websocket"])


@app.get("/")
def root():
    return {"message": "Mini Store Pick & Place API v1.0", "status": "operational"}


@app.get("/health")
def health():
    return {"status": "healthy"}
