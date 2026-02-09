"use client";

import { Job, JobStatus } from "@/lib/types";
import { cn } from "@/lib/utils";
import { Play, Loader2, CheckCircle2, AlertCircle, Coffee } from "lucide-react";

const STAGES: { key: JobStatus; label: string }[] = [
  { key: "planning", label: "AI Planning" },
  { key: "simulating", label: "Fluid Sim" },
  { key: "evaluating", label: "AI Eval" },
  { key: "completed", label: "Done" },
];

function getStageIndex(status: JobStatus): number {
  const idx = STAGES.findIndex((s) => s.key === status);
  return idx === -1 ? 0 : idx;
}

interface SimulationViewerProps {
  job: Job | null;
}

export function SimulationViewer({ job }: SimulationViewerProps) {
  if (!job) {
    return (
      <div className="flex flex-col items-center justify-center h-full min-h-[360px] rounded-2xl border-2 border-dashed border-stone-200 bg-stone-50/50">
        <Coffee className="w-16 h-16 text-stone-300 mb-4" />
        <p className="text-stone-400 text-sm font-medium">
          Select a pattern to start
        </p>
        <p className="text-stone-300 text-xs mt-1">
          Your simulation will appear here
        </p>
      </div>
    );
  }

  const currentStage = getStageIndex(job.status);
  const isProcessing = !["completed", "failed"].includes(job.status);

  return (
    <div className="space-y-4">
      {/* Video / Status Area */}
      <div className="relative rounded-2xl overflow-hidden bg-stone-900 min-h-[320px] flex items-center justify-center">
        {job.video_url && job.status === "completed" ? (
          <div className="w-full h-full flex items-center justify-center p-4">
            <div className="relative w-64 h-64 rounded-full bg-gradient-to-br from-amber-900 via-amber-800 to-stone-900 shadow-2xl flex items-center justify-center overflow-hidden">
              <div className="absolute inset-2 rounded-full bg-gradient-to-br from-amber-950 to-stone-950 opacity-60" />
              <div className="relative text-center">
                <CheckCircle2 className="w-10 h-10 text-amber-400 mx-auto mb-2" />
                <p className="text-amber-200 text-xs font-medium">
                  {job.pattern_name.charAt(0).toUpperCase() + job.pattern_name.slice(1)}
                </p>
                <p className="text-amber-400/60 text-[10px] mt-0.5">
                  Simulation Complete
                </p>
              </div>
            </div>
          </div>
        ) : job.status === "failed" ? (
          <div className="text-center p-8">
            <AlertCircle className="w-12 h-12 text-red-400 mx-auto mb-3" />
            <p className="text-red-300 text-sm font-medium">
              Simulation Failed
            </p>
            <p className="text-red-400/60 text-xs mt-1">
              {job.error_message || "An unexpected error occurred"}
            </p>
          </div>
        ) : (
          <div className="text-center p-8">
            <Loader2 className="w-12 h-12 text-amber-400 mx-auto mb-3 animate-spin" />
            <p className="text-amber-200 text-sm font-medium">
              {job.status === "planning" && "Gemini is analyzing your request..."}
              {job.status === "simulating" && "Running fluid simulation..."}
              {job.status === "evaluating" && "AI is evaluating the result..."}
              {job.status === "queued" && "Waiting in queue..."}
            </p>
          </div>
        )}
      </div>

      {/* Pipeline Progress */}
      <div className="flex items-center gap-1">
        {STAGES.map((stage, i) => {
          const isActive = stage.key === job.status;
          const isDone = currentStage > i || job.status === "completed";
          return (
            <div key={stage.key} className="flex-1 flex flex-col items-center gap-1.5">
              <div
                className={cn(
                  "w-full h-1.5 rounded-full transition-all duration-500",
                  isDone
                    ? "bg-amber-500"
                    : isActive
                    ? "bg-amber-400 animate-pulse"
                    : "bg-stone-200"
                )}
              />
              <span
                className={cn(
                  "text-[10px] font-medium transition-colors",
                  isDone
                    ? "text-amber-600"
                    : isActive
                    ? "text-amber-500"
                    : "text-stone-400"
                )}
              >
                {stage.label}
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
}
