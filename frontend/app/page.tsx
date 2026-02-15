"use client";

import { useState, useCallback, useEffect } from "react";
import { Bot, Sparkles, Trash2, Play, RotateCcw } from "lucide-react";
import { OrderInput } from "@/components/order-input";
import { OrderList } from "@/components/order-list";
import { PickListView } from "@/components/pick-list-view";
import { RobotStepLog } from "@/components/robot-step-log";
import { SimViewer } from "@/components/sim-viewer";
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
  const [simConnected, setSimConnected] = useState(false);

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
    startSim().then(() => setSimConnected(true)).catch(() => setSimConnected(false));
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

  const isRunning = activeOrder &&
    (activeOrder.status === "planning" || activeOrder.status === "picking");

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900">
      {/* Header */}
      <header className="border-b border-white/10 bg-black/20 backdrop-blur-sm">
        <div className="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between">
          <div className="flex items-center gap-4">
            <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-emerald-400 to-cyan-500 flex items-center justify-center shadow-lg shadow-emerald-500/25">
              <Trash2 className="w-6 h-6 text-white" />
            </div>
            <div>
              <h1 className="text-2xl font-bold text-white tracking-tight">
                Clutter Bot
              </h1>
              <p className="text-sm text-slate-400">
                AI-powered robotic cleanup assistant
              </p>
            </div>
          </div>
          <div className="flex items-center gap-6">
            <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-amber-500/10 border border-amber-500/20">
              <Sparkles className="w-4 h-4 text-amber-400" />
              <span className="text-sm text-amber-300 font-medium">Gemini 3</span>
            </div>
            <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-cyan-500/10 border border-cyan-500/20">
              <Bot className="w-4 h-4 text-cyan-400" />
              <span className="text-sm text-cyan-300 font-medium">MuJoCo Sim</span>
            </div>
          </div>
        </div>
      </header>

      <main className="max-w-7xl mx-auto px-6 py-8">
        {/* Status Bar */}
        <div className="mb-6 flex items-center gap-4">
          <div className={`flex items-center gap-2 px-4 py-2 rounded-xl ${
            simConnected
              ? "bg-emerald-500/10 border border-emerald-500/20"
              : "bg-red-500/10 border border-red-500/20"
          }`}>
            <div className={`w-2 h-2 rounded-full ${
              simConnected ? "bg-emerald-400 animate-pulse" : "bg-red-400"
            }`} />
            <span className={`text-sm font-medium ${
              simConnected ? "text-emerald-300" : "text-red-300"
            }`}>
              {simConnected ? "Sim Connected" : "Sim Disconnected"}
            </span>
          </div>
          {isRunning && (
            <div className="flex items-center gap-2 px-4 py-2 rounded-xl bg-blue-500/10 border border-blue-500/20">
              <Play className="w-4 h-4 text-blue-400 animate-pulse" />
              <span className="text-sm font-medium text-blue-300">Robot Working...</span>
            </div>
          )}
          <div className="ml-auto text-sm text-slate-500">
            Press <kbd className="px-2 py-0.5 rounded bg-slate-700 text-slate-300 font-mono text-xs">R</kbd> in sim to reset scene
          </div>
        </div>

        {/* Simulation Viewer */}
        <div className="mb-6">
          <SimViewer />
        </div>

        <div className="grid grid-cols-12 gap-6">
          {/* Left Column - Input & Orders */}
          <div className="col-span-4 space-y-6">
            <section className="rounded-2xl bg-slate-800/50 border border-white/10 p-6 backdrop-blur-sm">
              <OrderInput
                onSubmit={handleSubmit}
                isLoading={createOrder.isPending}
              />
            </section>
            <section className="rounded-2xl bg-slate-800/50 border border-white/10 p-6 backdrop-blur-sm">
              <OrderList
                orders={orders}
                activeOrderId={activeOrderId}
                onSelect={handleSelectOrder}
              />
            </section>
          </div>

          {/* Center Column - Robot Steps */}
          <div className="col-span-5">
            <section className="rounded-2xl bg-slate-800/50 border border-white/10 p-6 backdrop-blur-sm h-full">
              <div className="flex items-center justify-between mb-4">
                <h2 className="text-lg font-semibold text-white flex items-center gap-2">
                  <Bot className="w-5 h-5 text-cyan-400" />
                  Robot Activity
                </h2>
                {robotSteps.length > 0 && (
                  <button
                    onClick={() => setRobotSteps([])}
                    className="text-xs text-slate-400 hover:text-white transition flex items-center gap-1"
                  >
                    <RotateCcw className="w-3 h-3" />
                    Clear
                  </button>
                )}
              </div>
              <RobotStepLog
                steps={robotSteps}
                isActive={isRunning ?? false}
              />
            </section>
          </div>

          {/* Right Column - Pick List */}
          <div className="col-span-3 space-y-6">
            <section className="rounded-2xl bg-slate-800/50 border border-white/10 p-6 backdrop-blur-sm">
              <h2 className="text-lg font-semibold text-white mb-4 flex items-center gap-2">
                <Sparkles className="w-5 h-5 text-amber-400" />
                Objects to Tidy
              </h2>
              {activeOrder ? (
                <>
                  <div className="mb-4 p-3 rounded-xl bg-slate-700/50 border border-white/5">
                    <p className="text-sm text-slate-300 italic">
                      &ldquo;{activeOrder.natural_language_input}&rdquo;
                    </p>
                  </div>
                  <PickListView pickList={activeOrder.pick_list} />
                </>
              ) : (
                <p className="text-sm text-slate-500">Select a task to see objects.</p>
              )}
            </section>
            {activeOrder?.error_message && (
              <section className="rounded-2xl bg-red-500/10 border border-red-500/20 p-4">
                <p className="text-sm text-red-300">{activeOrder.error_message}</p>
              </section>
            )}
          </div>
        </div>
      </main>

      {/* Footer */}
      <footer className="mt-12 border-t border-white/10 bg-black/20">
        <div className="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between text-sm text-slate-500">
          <span>LabLab AI Hackathon - AI Meets Robotics</span>
          <span>Powered by Gemini + MuJoCo + Vultr</span>
        </div>
      </footer>
    </div>
  );
}
