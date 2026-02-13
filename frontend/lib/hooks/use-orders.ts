"use client";

import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";
import * as api from "../api";

export function useStoreItems() {
  return useQuery({
    queryKey: ["store-items"],
    queryFn: api.getStoreItems,
  });
}

export function useOrders() {
  return useQuery({
    queryKey: ["orders"],
    queryFn: api.listOrders,
    refetchInterval: 5000,
  });
}

export function useOrder(orderId: string | null) {
  return useQuery({
    queryKey: ["order", orderId],
    queryFn: () => api.getOrder(orderId!),
    enabled: !!orderId,
    refetchInterval: (query) => {
      const status = query.state.data?.status;
      return status === "completed" || status === "failed" ? false : 1000;
    },
  });
}

export function useCreateOrder() {
  const queryClient = useQueryClient();
  return useMutation({
    mutationFn: (naturalLanguageInput: string) => api.createOrder(naturalLanguageInput),
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: ["orders"] });
    },
  });
}
