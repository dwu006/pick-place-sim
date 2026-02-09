"use client";

import { useEffect, useRef, useCallback } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { Job } from "../types";

export function useJobWebSocket(jobId: string | null) {
  const wsRef = useRef<WebSocket | null>(null);
  const queryClient = useQueryClient();

  const connect = useCallback(() => {
    if (!jobId) return;

    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(`${protocol}//${window.location.host}/ws/${jobId}`);

    ws.onmessage = (event) => {
      const msg = JSON.parse(event.data);

      // Update the job query cache with new data
      queryClient.setQueryData<Job>(["job", jobId], (old) => {
        if (!old) return old;
        const updated = { ...old };

        switch (msg.type) {
          case "status_update":
            updated.status = msg.status;
            break;
          case "pour_spec_ready":
            updated.pour_spec = msg.pour_spec;
            break;
          case "simulation_complete":
            updated.video_url = msg.video_url;
            break;
          case "evaluation_complete":
            updated.status = "completed";
            updated.score = msg.score;
            updated.feedback = msg.feedback;
            updated.breakdown = msg.breakdown;
            break;
          case "error":
            updated.status = "failed";
            updated.error_message = msg.error;
            break;
        }

        return updated;
      });

      // Also invalidate the jobs list
      queryClient.invalidateQueries({ queryKey: ["jobs"] });
    };

    ws.onclose = () => {
      // Reconnect after 2s if job is still in progress
      const job = queryClient.getQueryData<Job>(["job", jobId]);
      if (job && job.status !== "completed" && job.status !== "failed") {
        setTimeout(connect, 2000);
      }
    };

    wsRef.current = ws;
  }, [jobId, queryClient]);

  useEffect(() => {
    connect();
    return () => {
      wsRef.current?.close();
    };
  }, [connect]);

  // Send a ping to keep connection alive
  useEffect(() => {
    const interval = setInterval(() => {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        wsRef.current.send("ping");
      }
    }, 30000);
    return () => clearInterval(interval);
  }, []);
}
