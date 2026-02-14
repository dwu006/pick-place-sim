"""
Single abstraction for order storage. When use_database is False, orders live in memory
(no SQLite). When True, we use SQLAlchemy. Lets you run the app without a DB for demos.
"""
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from config import settings

if settings.use_database:
    from database import async_session
    from models import Order as OrderModel


def _now() -> datetime:
    return datetime.now(timezone.utc)


# In-memory store when use_database is False
_memory_orders: Dict[str, Dict[str, Any]] = {}

# Wrist camera images per order_id (in-memory only; used for vision-based parsing)
_wrist_images: Dict[str, bytes] = {}


async def create_order(natural_language_input: str) -> Any:
    if settings.use_database:
        order = OrderModel(
            natural_language_input=natural_language_input,
            status="queued",
        )
        async with async_session() as db:
            db.add(order)
            await db.commit()
            await db.refresh(order)
        return order
    else:
        oid = str(uuid.uuid4())
        _memory_orders[oid] = {
            "id": oid,
            "natural_language_input": natural_language_input,
            "pick_list": None,
            "status": "queued",
            "error_message": None,
            "created_at": _now(),
            "updated_at": _now(),
        }
        return _memory_orders[oid]


async def get_order(order_id: str) -> Optional[Any]:
    if settings.use_database:
        async with async_session() as db:
            return await db.get(OrderModel, order_id)
    return _memory_orders.get(order_id)


async def list_orders(limit: int = 50) -> List[Any]:
    if settings.use_database:
        from sqlalchemy import select
        async with async_session() as db:
            result = await db.execute(
                select(OrderModel).order_by(OrderModel.created_at.desc()).limit(limit)
            )
            return list(result.scalars().all())
    items = sorted(
        _memory_orders.values(),
        key=lambda x: x["created_at"],
        reverse=True,
    )[:limit]
    return items


async def update_order(
    order_id: str,
    *,
    status: Optional[str] = None,
    pick_list: Optional[List[Dict[str, Any]]] = None,
    error_message: Optional[str] = None,
) -> None:
    if settings.use_database:
        async with async_session() as db:
            order = await db.get(OrderModel, order_id)
            if not order:
                return
            if status is not None:
                order.status = status
            if pick_list is not None:
                order.pick_list = pick_list
            if error_message is not None:
                order.error_message = error_message
            order.updated_at = _now()
            await db.commit()
    else:
        if order_id not in _memory_orders:
            return
        o = _memory_orders[order_id]
        if status is not None:
            o["status"] = status
        if pick_list is not None:
            o["pick_list"] = pick_list
        if error_message is not None:
            o["error_message"] = error_message
        o["updated_at"] = _now()


def set_wrist_image(order_id: str, image_bytes: bytes) -> None:
    """Store wrist image for an order (in-memory)."""
    _wrist_images[order_id] = image_bytes


def get_wrist_image(order_id: str) -> Optional[bytes]:
    """Return stored wrist image for an order, or None."""
    return _wrist_images.get(order_id)


async def get_next_queued_order() -> Optional[Any]:
    if settings.use_database:
        from sqlalchemy import select
        async with async_session() as db:
            result = await db.execute(
                select(OrderModel)
                .where(OrderModel.status == "queued")
                .order_by(OrderModel.created_at)
                .limit(1)
            )
            return result.scalar_one_or_none()
    for o in sorted(_memory_orders.values(), key=lambda x: x["created_at"]):
        if o["status"] == "queued":
            return o
    return None
