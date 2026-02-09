"use client";

import { useState } from "react";
import { Coffee, Zap, Brain } from "lucide-react";
import { PatternInput } from "@/components/pattern-input";
import { SimulationViewer } from "@/components/simulation-viewer";
import { EvaluationPanel } from "@/components/evaluation-panel";
import { PourSpecViewer } from "@/components/pour-spec-viewer";
import { JobList } from "@/components/job-list";
import { useCreateJob, useJob, useJobs } from "@/lib/hooks/use-jobs";
import { useJobWebSocket } from "@/lib/hooks/use-websocket";

export default function Dashboard() {
  const [activeJobId, setActiveJobId] = useState<string | null>(null);

  const createJob = useCreateJob();
  const { data: jobs = [] } = useJobs();
  const { data: activeJob = null } = useJob(activeJobId);

  // Connect WebSocket for active job
  useJobWebSocket(activeJobId);

  const handleSubmit = async (input: string, preset?: string) => {
    const job = await createJob.mutateAsync({ input, preset });
    setActiveJobId(job.id);
  };

  return (
    <div className="min-h-screen bg-stone-50">
      {/* Header */}
      <header className="border-b border-stone-200 bg-white">
        <div className="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="w-9 h-9 rounded-xl bg-gradient-to-br from-amber-600 to-amber-800 flex items-center justify-center shadow-md">
              <Coffee className="w-5 h-5 text-white" />
            </div>
            <div>
              <h1 className="text-lg font-bold text-stone-800 tracking-tight">
                LatteBot
              </h1>
              <p className="text-[10px] text-stone-400 -mt-0.5">
                Sim-to-Real Latte Art Training
              </p>
            </div>
          </div>
          <div className="flex items-center gap-4 text-xs text-stone-400">
            <div className="flex items-center gap-1.5">
              <Zap className="w-3.5 h-3.5 text-amber-500" />
              <span>Differentiable Physics</span>
            </div>
            <div className="flex items-center gap-1.5">
              <Brain className="w-3.5 h-3.5 text-blue-500" />
              <span>Gemini AI Eval</span>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-6 py-6">
        <div className="grid grid-cols-12 gap-6">
          {/* Left Column — Input & History */}
          <div className="col-span-3 space-y-6">
            <div className="rounded-2xl border-2 border-stone-100 bg-white p-5">
              <h2 className="text-sm font-semibold text-stone-700 mb-4">
                Create Latte Art
              </h2>
              <PatternInput
                onSubmit={handleSubmit}
                isLoading={createJob.isPending}
              />
            </div>

            <div className="rounded-2xl border-2 border-stone-100 bg-white p-5">
              <h2 className="text-sm font-semibold text-stone-700 mb-3">
                Recent Jobs
              </h2>
              <JobList
                jobs={jobs}
                activeJobId={activeJobId}
                onSelect={setActiveJobId}
              />
            </div>
          </div>

          {/* Center Column — Simulation */}
          <div className="col-span-5">
            <SimulationViewer job={activeJob} />
          </div>

          {/* Right Column — Parameters & Evaluation */}
          <div className="col-span-4 space-y-6">
            <PourSpecViewer pourSpec={activeJob?.pour_spec || null} />
            <EvaluationPanel job={activeJob} />
          </div>
        </div>
      </main>
    </div>
  );
}
