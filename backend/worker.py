import asyncio
import logging
from typing import Any

from config import settings
from order_store import get_next_queued_order, get_order, update_order
from routers.websocket import broadcast
from schemas import PickListItem
from services.gemini_order_parser import parse_order
from services.pick_place_simulator import run_pick_place

logger = logging.getLogger(__name__)


def _order_id(order: Any) -> str:
    return order.id if hasattr(order, "id") else order["id"]


async def _process_order(order_id: str):
    order = await get_order(order_id)
    if not order:
        return

    try:
        await update_order(order_id, status="planning")
        await broadcast(order_id, {"type": "status_update", "order_id": order_id, "status": "planning"})

        pick_list = await parse_order(order.natural_language_input if hasattr(order, "natural_language_input") else order["natural_language_input"])
        pick_list_dict = [{"item_id": p.item_id, "quantity": p.quantity} for p in pick_list]
        await update_order(order_id, pick_list=pick_list_dict)
        await broadcast(order_id, {"type": "pick_list_ready", "order_id": order_id, "pick_list": pick_list_dict})

        await update_order(order_id, status="picking")
        await broadcast(order_id, {"type": "status_update", "order_id": order_id, "status": "picking"})

        if getattr(settings, "use_gemini_robot_agent", False):
            from services.gemini_robot_agent import run_pick_place_with_gemini
            await run_pick_place_with_gemini(order_id, [PickListItem(**x) for x in pick_list_dict])
        else:
            await run_pick_place(order_id, [PickListItem(**x) for x in pick_list_dict])

        await update_order(order_id, status="completed")
        await broadcast(order_id, {"type": "order_complete", "order_id": order_id})

    except Exception as e:
        logger.exception("Order %s failed", order_id)
        await update_order(order_id, status="failed", error_message=str(e))
        await broadcast(order_id, {"type": "error", "order_id": order_id, "error": str(e)})


async def start_worker():
    logger.info("Worker started")
    while True:
        try:
            order = await get_next_queued_order()
            if order:
                await _process_order(_order_id(order))
            else:
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            logger.info("Worker shutting down")
            break
        except Exception:
            logger.exception("Worker error")
            await asyncio.sleep(2.0)
