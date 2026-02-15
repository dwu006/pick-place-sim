"use client";

import { useEffect, useRef, useState } from "react";

export function SimViewer() {
  const [connected, setConnected] = useState(false);
  const [wristFrame, setWristFrame] = useState<string | null>(null);
  const [freeFrame, setFreeFrame] = useState<string | null>(null);
  const [replayVideoUrl, setReplayVideoUrl] = useState<string | null>(null);
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
          // Clear video when new live frames arrive
          setReplayVideoUrl(null);
          // Check if it's dual camera data
          try {
            const frameData = JSON.parse(data.data);
            if (frameData.wrist && frameData.free) {
              setWristFrame(frameData.wrist);
              setFreeFrame(frameData.free);
            } else {
              // Fallback: single camera
              setWristFrame(data.data);
              setFreeFrame(null);
            }
          } catch {
            // Old format: single frame
            setWristFrame(data.data);
            setFreeFrame(null);
          }
        } else if (data.type === "video_ready" && data.video_url) {
          console.log("Replay video ready:", data.video_url);
          // Construct full URL
          const baseUrl = process.env.NEXT_PUBLIC_API_BASE || `http://${window.location.hostname}:8000`;
          setReplayVideoUrl(`${baseUrl}${data.video_url}`);
          // Clear live frames
          setWristFrame(null);
          setFreeFrame(null);
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
      <div className="relative h-96 bg-black flex items-center justify-center">
        {replayVideoUrl ? (
          <div className="w-full h-full flex flex-col items-center justify-center p-4">
            <div className="text-xs text-white/60 mb-2">Task Replay</div>
            <video
              src={replayVideoUrl}
              controls
              autoPlay
              loop
              className="max-w-full max-h-full rounded border border-white/10"
            />
          </div>
        ) : wristFrame || freeFrame ? (
          <div className="flex gap-2 w-full h-full p-2">
            {/* Wrist Camera - Left */}
            <div className="flex-1 flex flex-col items-center justify-center border border-white/10 rounded">
              {wristFrame ? (
                <>
                  <div className="text-xs text-white/40 mb-1">Wrist Camera</div>
                  <img
                    src={`data:image/png;base64,${wristFrame}`}
                    alt="Wrist camera view"
                    className="max-w-full max-h-full object-contain"
                  />
                </>
              ) : (
                <div className="text-white/30 text-xs">No wrist camera</div>
              )}
            </div>
            {/* Free Camera - Right */}
            <div className="flex-1 flex flex-col items-center justify-center border border-white/10 rounded">
              {freeFrame ? (
                <>
                  <div className="text-xs text-white/40 mb-1">Free Camera</div>
                  <img
                    src={`data:image/png;base64,${freeFrame}`}
                    alt="Free camera view"
                    className="max-w-full max-h-full object-contain"
                  />
                </>
              ) : (
                <div className="text-white/30 text-xs">No free camera</div>
              )}
            </div>
          </div>
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
