"use client";

import { PourSpec } from "@/lib/types";
import { ArrowUpDown, Waves, Droplets, ArrowDown, Settings2 } from "lucide-react";

const SPEC_FIELDS = [
  { key: "pour_height", label: "Pour Height", unit: "cm", icon: ArrowUpDown },
  { key: "oscillation_freq", label: "Oscillation", unit: "Hz", icon: Waves },
  { key: "flow_rate", label: "Flow Rate", unit: "ml/s", icon: Droplets },
  { key: "pull_through", label: "Pull Through", unit: "", icon: ArrowDown },
] as const;

interface PourSpecViewerProps {
  pourSpec: PourSpec | null;
}

export function PourSpecViewer({ pourSpec }: PourSpecViewerProps) {
  return (
    <div className="rounded-2xl border-2 border-stone-100 bg-white p-5">
      <div className="flex items-center gap-2 mb-4">
        <Settings2 className="w-4 h-4 text-amber-600" />
        <h3 className="text-sm font-semibold text-stone-700">Pour Parameters</h3>
        {pourSpec && (
          <span className="ml-auto text-[10px] bg-amber-50 text-amber-600 px-2 py-0.5 rounded-full font-medium capitalize">
            {pourSpec.pattern}
          </span>
        )}
      </div>

      {!pourSpec ? (
        <p className="text-stone-300 text-xs text-center py-4">
          Parameters will appear after AI planning
        </p>
      ) : (
        <div className="grid grid-cols-2 gap-3">
          {SPEC_FIELDS.map(({ key, label, unit, icon: Icon }) => {
            const val = pourSpec[key];
            return (
              <div
                key={key}
                className="bg-stone-50 rounded-xl p-3 flex flex-col gap-1"
              >
                <div className="flex items-center gap-1.5">
                  <Icon className="w-3.5 h-3.5 text-stone-400" />
                  <span className="text-[10px] text-stone-400 font-medium">
                    {label}
                  </span>
                </div>
                <span className="text-lg font-bold text-stone-800 tabular-nums">
                  {typeof val === "boolean" ? (val ? "Yes" : "No") : val}
                  {unit && (
                    <span className="text-xs text-stone-400 ml-0.5">
                      {unit}
                    </span>
                  )}
                </span>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}
