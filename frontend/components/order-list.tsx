"use client";

import { useState } from "react";
import { Order, OrderStatus } from "@/lib/types";
import { cn } from "@/lib/utils";
import { Clock, CheckCircle, XCircle, Loader2, Copy, Check, ListTodo } from "lucide-react";

const statusConfig: Record<
  OrderStatus,
  { label: string; icon: typeof Clock; className: string; bgClass: string }
> = {
  queued: {
    label: "Queued",
    icon: Clock,
    className: "text-slate-400",
    bgClass: "bg-slate-600/50",
  },
  planning: {
    label: "Planning",
    icon: Loader2,
    className: "text-amber-400",
    bgClass: "bg-amber-500/20",
  },
  picking: {
    label: "Picking",
    icon: Loader2,
    className: "text-cyan-400",
    bgClass: "bg-cyan-500/20",
  },
  completed: {
    label: "Done",
    icon: CheckCircle,
    className: "text-emerald-400",
    bgClass: "bg-emerald-500/20",
  },
  failed: {
    label: "Failed",
    icon: XCircle,
    className: "text-red-400",
    bgClass: "bg-red-500/20",
  },
};

interface OrderListProps {
  orders: Order[];
  activeOrderId: string | null;
  onSelect: (id: string) => void;
}

export function OrderList({ orders, activeOrderId, onSelect }: OrderListProps) {
  const [copiedId, setCopiedId] = useState<string | null>(null);

  const copyOrderId = (e: React.MouseEvent, id: string) => {
    e.stopPropagation();
    navigator.clipboard.writeText(id);
    setCopiedId(id);
    setTimeout(() => setCopiedId(null), 2000);
  };

  return (
    <div className="space-y-3">
      <h2 className="text-sm font-semibold text-slate-300 flex items-center gap-2">
        <ListTodo className="w-4 h-4 text-cyan-400" />
        Recent Tasks
      </h2>
      {orders.length === 0 ? (
        <p className="text-sm text-slate-500 italic py-4">No tasks yet. Try one above!</p>
      ) : (
        <ul className="space-y-2 max-h-64 overflow-y-auto">
          {orders.map((order) => {
            const config = statusConfig[order.status];
            const Icon = config.icon;
            return (
              <li key={order.id}>
                <div
                  onClick={() => onSelect(order.id)}
                  className={cn(
                    "w-full text-left px-4 py-3 rounded-xl border transition-all flex items-center gap-3 cursor-pointer",
                    activeOrderId === order.id
                      ? "border-emerald-500/50 bg-emerald-500/10"
                      : "border-white/5 hover:border-white/10 hover:bg-slate-700/30"
                  )}
                >
                  <div className="flex-1 min-w-0">
                    <p className="text-sm text-white truncate">
                      {order.natural_language_input || "—"}
                    </p>
                    <div className="flex items-center gap-2 mt-1.5">
                      <span
                        className={cn(
                          "inline-flex items-center gap-1 px-2 py-0.5 rounded-full text-xs font-medium",
                          config.bgClass,
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
                  <button
                    type="button"
                    onClick={(e) => copyOrderId(e, order.id)}
                    className="shrink-0 p-2 rounded-lg text-slate-500 hover:bg-slate-600/50 hover:text-white transition"
                    title="Copy order ID"
                  >
                    {copiedId === order.id ? <Check className="w-4 h-4 text-emerald-400" /> : <Copy className="w-4 h-4" />}
                  </button>
                </div>
              </li>
            );
          })}
        </ul>
      )}
    </div>
  );
}
