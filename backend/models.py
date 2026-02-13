import uuid
from datetime import datetime, timezone
from typing import Optional

from sqlalchemy import JSON, Index, String, Text
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


class Base(DeclarativeBase):
    pass


class Order(Base):
    __tablename__ = "orders"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    natural_language_input: Mapped[str] = mapped_column(Text, nullable=False)
    pick_list: Mapped[Optional[dict]] = mapped_column(JSON, nullable=True)  # [{"item_id": str, "quantity": int}]
    status: Mapped[str] = mapped_column(String, nullable=False, default="queued")
    error_message: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    created_at: Mapped[datetime] = mapped_column(default=lambda: datetime.now(timezone.utc))
    updated_at: Mapped[datetime] = mapped_column(
        default=lambda: datetime.now(timezone.utc),
        onupdate=lambda: datetime.now(timezone.utc),
    )

    __table_args__ = (
        Index("idx_orders_status", "status"),
        Index("idx_orders_created_at", "created_at"),
    )
