from typing import Any, List

from fastapi import APIRouter, HTTPException

from order_store import create_order as store_create_order, get_order as store_get_order, list_orders as store_list_orders
from schemas import OrderCreateRequest, OrderResponse, StoreItem

router = APIRouter()

STORE_ITEMS = [
    StoreItem(id="apple", name="Apple", description="Fresh apple"),
    StoreItem(id="banana", name="Banana", description="Fresh banana"),
    StoreItem(id="water", name="Water bottle", description="Bottled water"),
    StoreItem(id="chips", name="Chips", description="Crispy chips"),
    StoreItem(id="soda", name="Soda", description="Can of soda"),
    StoreItem(id="cookie", name="Cookie", description="Cookie"),
    StoreItem(id="sandwich", name="Sandwich", description="Sandwich"),
    StoreItem(id="coffee", name="Coffee", description="Coffee"),
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
    return STORE_ITEMS
