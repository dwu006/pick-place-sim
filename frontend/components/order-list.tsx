"use client";

import { Order, OrderStatus } from "@/lib/types";
import { cn } from "@/lib/utils";
import { Package, Clock, CheckCircle, XCircle, Loader2 } from "lucide-react";

const statusConfig: Record<
  OrderStatus,
  { label: string; icon: typeof Package; className: string }
> = {
  queued: {
    label: "Queued",
    icon: Clock,
    className: "bg-stone-100 text-stone-600",
  },
  planning: {
    label: "Planning",
    icon: Loader2,
    className: "bg-amber-100 text-amber-800",
  },
  picking: {
    label: "Picking",
    icon: Loader2,
    className: "bg-blue-100 text-blue-800",
  },
  completed: {
    label: "Done",
    icon: CheckCircle,
    className: "bg-emerald-100 text-emerald-800",
  },
  failed: {
    label: "Failed",
    icon: XCircle,
    className: "bg-red-100 text-red-800",
  },
};

interface OrderListProps {
  orders: Order[];
  activeOrderId: string | null;
  onSelect: (id: string) => void;
}

export function OrderList({ orders, activeOrderId, onSelect }: OrderListProps) {
  return (
    <div className="space-y-2">
      <h2 className="text-sm font-semibold text-stone-700">Recent tasks</h2>
      <ul className="space-y-1.5 max-h-64 overflow-y-auto">
        {orders.map((order) => {
          const config = statusConfig[order.status];
          const Icon = config.icon;
          return (
            <li key={order.id}>
              <button
                type="button"
                onClick={() => onSelect(order.id)}
                className={cn(
                  "w-full text-left px-3 py-2.5 rounded-xl border-2 transition-all flex items-center gap-3",
                  activeOrderId === order.id
                    ? "border-emerald-500 bg-emerald-50"
                    : "border-stone-100 hover:border-stone-200 hover:bg-stone-50"
                )}
              >
                <div className="flex-1 min-w-0">
                  <p className="text-sm text-stone-800 truncate">
                    {order.natural_language_input || "—"}
                  </p>
                  <div className="flex items-center gap-2 mt-1">
                    <span
                      className={cn(
                        "inline-flex items-center gap-1 px-1.5 py-0.5 rounded text-xs font-medium",
                        config.className
                      )}
                    >
                      {order.status === "planning" || order.status === "picking" ? (
                        <Icon className="w-3 h-3 animate-spin" />
                      ) : (
                        <Icon className="w-3 h-3" />
                      )}
                      {config.label}
                    </span>
                  </div>
                </div>
              </button>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
