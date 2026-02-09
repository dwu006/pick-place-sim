"use client";

import { Job } from "@/lib/types";
import { cn } from "@/lib/utils";
import { Star, MessageSquare, BarChart3 } from "lucide-react";

const METRIC_LABELS: Record<string, string> = {
  contrast: "Contrast",
  symmetry: "Symmetry",
  definition: "Definition",
  crema_quality: "Crema Quality",
};

interface EvaluationPanelProps {
  job: Job | null;
}

function scoreColor(score: number): string {
  if (score >= 90) return "text-green-500";
  if (score >= 80) return "text-amber-500";
  if (score >= 70) return "text-orange-500";
  return "text-red-500";
}

function barColor(score: number): string {
  if (score >= 90) return "bg-green-500";
  if (score >= 80) return "bg-amber-500";
  if (score >= 70) return "bg-orange-500";
  return "bg-red-500";
}

export function EvaluationPanel({ job }: EvaluationPanelProps) {
  if (!job || !job.score) {
    return (
      <div className="rounded-2xl border-2 border-stone-100 bg-white p-5">
        <div className="flex items-center gap-2 mb-4">
          <BarChart3 className="w-4 h-4 text-stone-400" />
          <h3 className="text-sm font-semibold text-stone-500">AI Evaluation</h3>
        </div>
        <p className="text-stone-300 text-xs text-center py-6">
          Evaluation results will appear after simulation completes
        </p>
      </div>
    );
  }

  return (
    <div className="rounded-2xl border-2 border-stone-100 bg-white p-5 space-y-4">
      {/* Header */}
      <div className="flex items-center gap-2">
        <BarChart3 className="w-4 h-4 text-amber-600" />
        <h3 className="text-sm font-semibold text-stone-700">AI Evaluation</h3>
        <span className="ml-auto text-[10px] bg-amber-50 text-amber-600 px-2 py-0.5 rounded-full font-medium">
          Gemini Pro
        </span>
      </div>

      {/* Score */}
      <div className="text-center py-2">
        <div className={cn("text-5xl font-bold tabular-nums", scoreColor(job.score))}>
          {job.score.toFixed(1)}
        </div>
        <div className="flex items-center justify-center gap-1 mt-1">
          {[...Array(5)].map((_, i) => (
            <Star
              key={i}
              className={cn(
                "w-3.5 h-3.5",
                i < Math.round(job.score / 20)
                  ? "text-amber-400 fill-amber-400"
                  : "text-stone-200"
              )}
            />
          ))}
        </div>
      </div>

      {/* Breakdown Bars */}
      {job.breakdown && (
        <div className="space-y-2.5">
          {Object.entries(job.breakdown).map(([key, value]) => (
            <div key={key}>
              <div className="flex justify-between text-xs mb-1">
                <span className="text-stone-500 font-medium">
                  {METRIC_LABELS[key] || key}
                </span>
                <span className="text-stone-700 font-semibold tabular-nums">
                  {value.toFixed(1)}
                </span>
              </div>
              <div className="h-1.5 bg-stone-100 rounded-full overflow-hidden">
                <div
                  className={cn("h-full rounded-full transition-all duration-700", barColor(value))}
                  style={{ width: `${value}%` }}
                />
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Feedback */}
      {job.feedback && (
        <div className="pt-2 border-t border-stone-100">
          <div className="flex items-start gap-2">
            <MessageSquare className="w-3.5 h-3.5 text-amber-500 mt-0.5 flex-shrink-0" />
            <p className="text-xs text-stone-600 leading-relaxed">{job.feedback}</p>
          </div>
        </div>
      )}
    </div>
  );
}
