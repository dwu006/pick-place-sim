"use client";

import { useRef, useEffect } from "react";
import { cn } from "@/lib/utils";
import { Package, ArrowRight, Check, Hand, MapPin } from "lucide-react";

interface StepEntry {
  step: string;
  item_id?: string;
  message: string;
  at: number;
}

interface RobotStepLogProps {
  steps: StepEntry[];
  isActive: boolean;
}

const stepConfig: Record<string, { icon: typeof Package; color: string; bg: string }> = {
  move_to_pick: { icon: MapPin, color: "text-blue-400", bg: "bg-blue-500/10 border-blue-500/20" },
  pick: { icon: Hand, color: "text-amber-400", bg: "bg-amber-500/10 border-amber-500/20" },
  move_to_delivery: { icon: ArrowRight, color: "text-purple-400", bg: "bg-purple-500/10 border-purple-500/20" },
  place: { icon: Package, color: "text-cyan-400", bg: "bg-cyan-500/10 border-cyan-500/20" },
  done: { icon: Check, color: "text-emerald-400", bg: "bg-emerald-500/10 border-emerald-500/20" },
  planning: { icon: Package, color: "text-slate-400", bg: "bg-slate-500/10 border-slate-500/20" },
};

export function RobotStepLog({ steps, isActive }: RobotStepLogProps) {
  const endRef = useRef<HTMLDivElement>(null);

  // Auto-scroll disabled to allow user to browse page while robot is active
  // useEffect(() => {
  //   endRef.current?.scrollIntoView({ behavior: "smooth", block: "nearest", inline: "nearest" });
  // }, [steps.length]);

  if (steps.length === 0 && !isActive) {
    return (
      <div className="text-sm text-slate-500 italic py-8 text-center">
        Select a task to see robot activity.
      </div>
    );
  }

  if (steps.length === 0) {
    return (
      <div className="text-sm text-slate-400 py-8 flex flex-col items-center gap-3">
        <div className="w-8 h-8 border-2 border-cyan-500 border-t-transparent rounded-full animate-spin" />
        <span>Waiting for robot steps...</span>
      </div>
    );
  }

  return (
    <div className="space-y-2 max-h-80 overflow-y-auto">
      {steps.map((s, i) => {
        const config = stepConfig[s.step] ?? stepConfig.planning;
        const Icon = config.icon;
        return (
          <div
            key={i}
            className={cn(
              "flex items-center gap-3 px-4 py-3 rounded-xl text-sm border transition-all",
              config.bg
            )}
          >
            <div className={cn("p-1.5 rounded-lg", config.bg)}>
              <Icon className={cn("w-4 h-4", config.color)} />
            </div>
            <span className="text-slate-200">{s.message}</span>
          </div>
        );
      })}
      <div ref={endRef} />
    </div>
  );
}
