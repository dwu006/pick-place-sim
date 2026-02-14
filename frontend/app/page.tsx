"use client";

import { useState, useCallback, useEffect } from "react";
import { LayoutGrid, Bot, Sparkles } from "lucide-react";
import { OrderInput } from "@/components/order-input";
import { OrderList } from "@/components/order-list";
import { PickListView } from "@/components/pick-list-view";
import { RobotStepLog } from "@/components/robot-step-log";
import { useCreateOrder, useOrder, useOrders } from "@/lib/hooks/use-orders";
import { useOrderWebSocket } from "@/lib/hooks/use-websocket";
import { startSim } from "@/lib/api";

interface StepEntry {
  step: string;
  item_id?: string;
  message: string;
  at: number;
}

export default function CleanupRoomPage() {
  const [activeOrderId, setActiveOrderId] = useState<string | null>(null);
  const [robotSteps, setRobotSteps] = useState<StepEntry[]>([]);

  const createOrder = useCreateOrder();
  const { data: orders = [] } = useOrders();
  const { data: activeOrder = null } = useOrder(activeOrderId);

  const handleWsMessage = useCallback((msg: Record<string, unknown>) => {
    if (msg.type === "robot_step" && typeof msg.message === "string") {
      const newStep: StepEntry = {
        step: String(msg.step ?? ""),
        item_id: msg.item_id ? String(msg.item_id) : undefined,
        message: msg.message,
        at: Date.now(),
      };
      setRobotSteps((prev) => [...prev, newStep]);
    }
  }, []);

  useOrderWebSocket(activeOrderId, handleWsMessage);

  useEffect(() => {
    startSim().catch(() => {});
  }, []);

  const handleSubmit = async (input: string) => {
    const order = await createOrder.mutateAsync(input);
    setActiveOrderId(order.id);
    setRobotSteps([]);
  };

  const handleSelectOrder = (id: string) => {
    setActiveOrderId(id);
    setRobotSteps([]);
  };

  return (
    <div className="min-h-screen bg-stone-50">
      <header className="border-b border-stone-200 bg-white">
        <div className="max-w-6xl mx-auto px-6 py-4 flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 rounded-xl bg-emerald-600 flex items-center justify-center shadow-md">
              <LayoutGrid className="w-5 h-5 text-white" />
            </div>
            <div>
              <h1 className="text-xl font-bold text-stone-800 tracking-tight">
                Cleanup Room
              </h1>
              <p className="text-xs text-stone-500 -mt-0.5">
                Say what to tidy — robot picks up & puts in bin
              </p>
            </div>
          </div>
          <div className="flex items-center gap-4 text-xs text-stone-500">
            <span className="flex items-center gap-1.5">
              <Sparkles className="w-4 h-4 text-amber-500" />
              Gemini
            </span>
            <span className="flex items-center gap-1.5">
              <Bot className="w-4 h-4 text-blue-500" />
              Sim pick & place in bin
            </span>
          </div>
        </div>
      </header>

      <main className="max-w-6xl mx-auto px-6 py-6">
        <div className="grid grid-cols-12 gap-6">
          <div className="col-span-4 space-y-6">
            <section className="rounded-2xl border-2 border-stone-100 bg-white p-5">
              <OrderInput
                onSubmit={handleSubmit}
                isLoading={createOrder.isPending}
              />
            </section>
            <section className="rounded-2xl border-2 border-stone-100 bg-white p-5">
              <OrderList
                orders={orders}
                activeOrderId={activeOrderId}
                onSelect={handleSelectOrder}
              />
            </section>
          </div>

          <div className="col-span-5 rounded-2xl border-2 border-stone-100 bg-white p-5">
            <h2 className="text-sm font-semibold text-stone-700 mb-3">
              Robot steps
            </h2>
            <RobotStepLog
              steps={robotSteps}
              isActive={
                !!activeOrder &&
                activeOrder.status !== "completed" &&
                activeOrder.status !== "failed"
              }
            />
          </div>

          <div className="col-span-3 space-y-6">
            <section className="rounded-2xl border-2 border-stone-100 bg-white p-5">
              <h2 className="text-sm font-semibold text-stone-700 mb-3">
                Objects to tidy
              </h2>
              {activeOrder ? (
                <>
                  <p className="text-sm text-stone-600 mb-3 italic">
                    &ldquo;{activeOrder.natural_language_input}&rdquo;
                  </p>
                  <PickListView pickList={activeOrder.pick_list} />
                </>
              ) : (
                <p className="text-sm text-stone-400">Select a task.</p>
              )}
            </section>
            {activeOrder?.error_message && (
              <section className="rounded-2xl border-2 border-red-100 bg-red-50 p-4">
                <p className="text-sm text-red-800">{activeOrder.error_message}</p>
              </section>
            )}
          </div>
        </div>
      </main>
    </div>
  );
}
