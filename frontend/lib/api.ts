import { Job, PatternPreset } from "./types";

const BASE = "/api";

async function request<T>(path: string, options?: RequestInit): Promise<T> {
  const res = await fetch(`${BASE}${path}`, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || "Request failed");
  }
  return res.json();
}

export async function getPatterns(): Promise<PatternPreset[]> {
  return request("/patterns");
}

export async function createJob(
  naturalLanguageInput: string,
  patternPreset?: string
): Promise<Job> {
  return request("/jobs", {
    method: "POST",
    body: JSON.stringify({
      natural_language_input: naturalLanguageInput,
      pattern_preset: patternPreset || undefined,
    }),
  });
}

export async function getJob(jobId: string): Promise<Job> {
  return request(`/jobs/${jobId}`);
}

export async function listJobs(): Promise<Job[]> {
  return request("/jobs");
}
