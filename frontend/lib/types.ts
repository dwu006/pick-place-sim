export type JobStatus =
  | "queued"
  | "planning"
  | "simulating"
  | "evaluating"
  | "completed"
  | "failed";

export interface PourSpec {
  pattern: string;
  pour_height: number;
  oscillation_freq: number;
  flow_rate: number;
  pull_through: boolean;
}

export interface Job {
  id: string;
  pattern_name: string;
  natural_language_input: string;
  pour_spec: PourSpec | null;
  status: JobStatus;
  video_url: string | null;
  score: number | null;
  feedback: string | null;
  breakdown: Record<string, number> | null;
  error_message: string | null;
  created_at: string;
  updated_at: string;
}

export interface PatternPreset {
  id: string;
  name: string;
  description: string;
}

export interface EvaluationResult {
  score: number;
  feedback: string;
  breakdown: Record<string, number>;
}
