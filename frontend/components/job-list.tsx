"use client";

import { Job } from "@/lib/types";
import { cn } from "@/lib/utils";
import { Clock, CheckCircle2, Loader2, AlertCircle } from "lucide-react";

function StatusIcon({ status }: { status: string }) {
  switch (status) {
    case "completed":
      return <CheckCircle2 className="w-3.5 h-3.5 text-green-500" />;
    case "failed":
      return <AlertCircle className="w-3.5 h-3.5 text-red-500" />;
    case "queued":
      return <Clock className="w-3.5 h-3.5 text-stone-400" />;
    default:
      return <Loader2 className="w-3.5 h-3.5 text-amber-500 animate-spin" />;
  }
}

function timeAgo(dateStr: string): string {
  const diff = Date.now() - new Date(dateStr).getTime();
  const mins = Math.floor(diff / 60000);
  if (mins < 1) return "just now";
  if (mins < 60) return `${mins}m ago`;
  const hrs = Math.floor(mins / 60);
  if (hrs < 24) return `${hrs}h ago`;
  return `${Math.floor(hrs / 24)}d ago`;
}

interface JobListProps {
  jobs: Job[];
  activeJobId: string | null;
  onSelect: (jobId: string) => void;
}

export function JobList({ jobs, activeJobId, onSelect }: JobListProps) {
  if (jobs.length === 0) {
    return (
      <p className="text-stone-300 text-xs text-center py-4">
        No jobs yet. Create one above!
      </p>
    );
  }

  return (
    <div className="space-y-1.5 max-h-[280px] overflow-y-auto pr-1">
      {jobs.map((job) => (
        <button
          key={job.id}
          onClick={() => onSelect(job.id)}
          className={cn(
            "w-full text-left p-3 rounded-xl transition-all flex items-center gap-3",
            activeJobId === job.id
              ? "bg-amber-50 border-2 border-amber-200"
              : "bg-stone-50 border-2 border-transparent hover:border-stone-200"
          )}
        >
          <StatusIcon status={job.status} />
          <div className="flex-1 min-w-0">
            <p className="text-xs font-medium text-stone-700 truncate">
              {job.natural_language_input}
            </p>
            <p className="text-[10px] text-stone-400 mt-0.5 capitalize">
              {job.pattern_name} &middot; {timeAgo(job.created_at)}
            </p>
          </div>
          {job.score && (
            <span className="text-xs font-bold text-amber-600 tabular-nums">
              {job.score.toFixed(0)}
            </span>
          )}
        </button>
      ))}
    </div>
  );
}
