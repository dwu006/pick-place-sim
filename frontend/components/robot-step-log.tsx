"use client";

import { useRef, useEffect } from "react";
import { cn } from "@/lib/utils";
import { Package, ArrowRight, Check } from "lucide-react";

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

const stepIcons: Record<string, typeof Package> = {
  move_to_pick: Package,
  pick: Package,
  move_to_delivery: ArrowRight,
  place: Check,
  done: Check,
};

export function RobotStepLog({ steps, isActive }: RobotStepLogProps) {
  const endRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [steps.length]);

  if (steps.length === 0 && !isActive) {
    return (
      <div className="text-sm text-stone-400 italic py-4">
        Select an order to see robot steps.
      </div>
    );
  }

  if (steps.length === 0) {
    return (
      <div className="text-sm text-stone-500 py-4 flex items-center gap-2">
        <div className="w-4 h-4 border-2 border-emerald-500 border-t-transparent rounded-full animate-spin" />
        Waiting for robot steps…
      </div>
    );
  }

  return (
    <div className="space-y-1.5 max-h-72 overflow-y-auto">
      {steps.map((s, i) => {
        const Icon = stepIcons[s.step] ?? Package;
        return (
          <div
            key={i}
            className={cn(
              "flex items-center gap-3 px-3 py-2 rounded-lg text-sm",
              s.step === "done"
                ? "bg-emerald-50 text-emerald-800"
                : "bg-stone-50 text-stone-700"
            )}
          >
            <Icon
              className={cn(
                "w-4 h-4 shrink-0",
                s.step === "done" ? "text-emerald-600" : "text-stone-400"
              )}
            />
            <span>{s.message}</span>
          </div>
        );
      })}
      <div ref={endRef} />
    </div>
  );
}
