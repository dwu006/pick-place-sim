from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel


class PickListItem(BaseModel):
    item_id: str
    quantity: int


class OrderCreateRequest(BaseModel):
    natural_language_input: str


class OrderResponse(BaseModel):
    id: str
    natural_language_input: str
    pick_list: Optional[List[PickListItem]] = None
    status: str
    error_message: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class StoreItem(BaseModel):
    id: str
    name: str
    description: str
