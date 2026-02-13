import { Order, StoreItem } from "./types";

const API_BASE = process.env.NEXT_PUBLIC_API_BASE ?? "";
const BASE = API_BASE ? `${API_BASE.replace(/\/$/, "")}/api` : "/api";

/** Base URL for WebSocket (same host as API). Use when backend is on another origin (e.g. Vultr). */
export function getWsBase(): string {
  if (typeof window === "undefined") return "";
  if (API_BASE) {
    const u = new URL(API_BASE);
    return (u.protocol === "https:" ? "wss:" : "ws:") + "//" + u.host;
  }
  return (window.location.protocol === "https:" ? "wss:" : "ws:") + "//" + window.location.host;
}

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

export async function getStoreItems(): Promise<StoreItem[]> {
  return request("/store/items");
}

export async function createOrder(naturalLanguageInput: string): Promise<Order> {
  return request("/orders", {
    method: "POST",
    body: JSON.stringify({
      natural_language_input: naturalLanguageInput,
    }),
  });
}

export async function getOrder(orderId: string): Promise<Order> {
  return request(`/orders/${orderId}`);
}

export async function listOrders(): Promise<Order[]> {
  return request("/orders");
}
