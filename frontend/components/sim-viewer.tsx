"use client";

import { useEffect, useRef, useState } from "react";

export function SimViewer() {
  const [connected, setConnected] = useState(false);
  const [frameData, setFrameData] = useState<string | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const wsUrl = process.env.NEXT_PUBLIC_API_BASE
      ? `${process.env.NEXT_PUBLIC_API_BASE.replace("http://", "ws://").replace("https://", "wss://")}/ws/viewer`
      : `ws://${window.location.hostname}:8000/ws/viewer`;

    console.log("Connecting to sim viewer WebSocket:", wsUrl);

    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;

    ws.onopen = () => {
      console.log("SimViewer WebSocket connected");
      setConnected(true);
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === "frame" && data.data) {
          setFrameData(data.data);
        }
      } catch (err) {
        console.error("Failed to parse sim viewer message:", err);
      }
    };

    ws.onerror = (error) => {
      console.error("SimViewer WebSocket error:", error);
      setConnected(false);
    };

    ws.onclose = () => {
      console.log("SimViewer WebSocket closed");
      setConnected(false);
    };

    return () => {
      ws.close();
    };
  }, []);

  return (
    <div className="border border-white/10 rounded-lg overflow-hidden bg-black/20">
      <div className="p-3 border-b border-white/10 flex items-center justify-between">
        <h3 className="font-medium text-sm">Simulation View</h3>
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${connected ? "bg-green-500" : "bg-red-500"}`} />
          <span className="text-xs text-white/60">{connected ? "Connected" : "Disconnected"}</span>
        </div>
      </div>
      <div className="relative aspect-[4/3] bg-black flex items-center justify-center">
        {frameData ? (
          <img
            src={`data:image/png;base64,${frameData}`}
            alt="Simulation view"
            className="w-full h-full object-contain"
          />
        ) : (
          <div className="text-white/40 text-sm text-center p-8">
            {connected ? "Waiting for simulation frames..." : "Simulation not running"}
            <div className="mt-2 text-xs text-white/30">
              Start the sim client with --enable-web-viewer flag
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
