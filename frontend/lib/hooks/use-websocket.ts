"use client";

import { useEffect, useRef, useCallback } from "react";
import { useQueryClient } from "@tanstack/react-query";
import { getWsBase } from "../api";
import { Order } from "../types";

type WsMessage = Record<string, unknown>;

export function useOrderWebSocket(
  orderId: string | null,
  onMessage?: (msg: WsMessage) => void
) {
  const wsRef = useRef<WebSocket | null>(null);
  const queryClient = useQueryClient();
  const onMessageRef = useRef(onMessage);
  onMessageRef.current = onMessage;

  const connect = useCallback(() => {
    if (!orderId) return;

    const base = getWsBase();
    const ws = new WebSocket(`${base}/ws/${orderId}`);

    ws.onmessage = (event) => {
      const msg = JSON.parse(event.data) as WsMessage;
      onMessageRef.current?.(msg);

      queryClient.setQueryData<Order>(["order", orderId], (old) => {
        if (!old) return old;
        const updated = { ...old };
        switch (msg.type) {
          case "status_update":
            updated.status = msg.status as Order["status"];
            break;
          case "pick_list_ready":
            updated.pick_list = msg.pick_list as Order["pick_list"];
            break;
          case "order_complete":
            updated.status = "completed";
            break;
          case "error":
            updated.status = "failed";
            updated.error_message = msg.error as string;
            break;
        }
        return updated;
      });

      queryClient.invalidateQueries({ queryKey: ["orders"] });
    };

    ws.onclose = () => {
      const order = queryClient.getQueryData<Order>(["order", orderId]);
      if (order && order.status !== "completed" && order.status !== "failed") {
        setTimeout(connect, 2000);
      }
    };

    wsRef.current = ws;
  }, [orderId, queryClient]);

  useEffect(() => {
    connect();
    return () => {
      wsRef.current?.close();
    };
  }, [connect]);

  useEffect(() => {
    const interval = setInterval(() => {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        wsRef.current.send("ping");
      }
    }, 30000);
    return () => clearInterval(interval);
  }, []);
}
