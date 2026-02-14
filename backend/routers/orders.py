from typing import Any, List

from fastapi import APIRouter, HTTPException

from order_store import create_order as store_create_order, get_order as store_get_order, list_orders as store_list_orders
from schemas import OrderCreateRequest, OrderResponse, StoreItem

router = APIRouter()

# Room objects the robot can pick up and put in the bin (must match gemini_order_parser ROOM_OBJECT_IDS)
ROOM_OBJECTS = [
    StoreItem(id="red_block", name="Red block", description="Red block on table"),
    StoreItem(id="blue_block", name="Blue block", description="Blue block on table"),
    StoreItem(id="green_block", name="Green block", description="Green block on table"),
    StoreItem(id="cup", name="Cup", description="Cup on table"),
    StoreItem(id="bottle", name="Bottle", description="Bottle on table"),
    StoreItem(id="toy", name="Toy", description="Toy to tidy"),
    StoreItem(id="book", name="Book", description="Book to tidy"),
    StoreItem(id="box", name="Box", description="Box to tidy"),
]


def _to_response(order: Any) -> OrderResponse:
    """Turn DB model or in-memory dict into OrderResponse."""
    if hasattr(order, "id"):
        return OrderResponse.model_validate(order)
    return OrderResponse(
        id=order["id"],
        natural_language_input=order["natural_language_input"],
        pick_list=order.get("pick_list"),
        status=order["status"],
        error_message=order.get("error_message"),
        created_at=order["created_at"],
        updated_at=order["updated_at"],
    )


@router.post("/orders", response_model=OrderResponse, status_code=201)
async def create_order(req: OrderCreateRequest):
    order = await store_create_order(req.natural_language_input)
    return _to_response(order)


@router.get("/orders", response_model=List[OrderResponse])
async def list_orders():
    orders = await store_list_orders(50)
    return [_to_response(o) for o in orders]


@router.get("/orders/{order_id}", response_model=OrderResponse)
async def get_order(order_id: str):
    order = await store_get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    return _to_response(order)


@router.get("/store/items", response_model=List[StoreItem])
async def list_store_items():
    """List room objects that can be picked up (cleanup task)."""
    return ROOM_OBJECTS
