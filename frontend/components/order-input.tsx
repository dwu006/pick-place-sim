"use client";

import { useState } from "react";
import { Send, Loader2 } from "lucide-react";
import { cn } from "@/lib/utils";

interface OrderInputProps {
  onSubmit: (input: string) => void;
  isLoading: boolean;
}

export function OrderInput({ onSubmit, isLoading }: OrderInputProps) {
  const [input, setInput] = useState("");

  const handleSubmit = () => {
    if (!input.trim() || isLoading) return;
    onSubmit(input.trim());
    setInput("");
  };

  return (
    <div className="space-y-3">
      <label className="text-sm font-medium text-stone-600">
        What should the robot clean up?
      </label>
      <div className="relative">
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder='e.g. "Pick up the duck and banana" or "Clean up all the toys"'
          className={cn(
            "w-full px-4 py-3 pr-12 rounded-xl border-2 text-stone-800 placeholder:text-stone-400",
            "focus:outline-none focus:border-emerald-500 focus:ring-2 focus:ring-emerald-500/20 transition-all",
            "bg-white border-stone-200"
          )}
          onKeyDown={(e) => {
            if (e.key === "Enter") {
              e.preventDefault();
              handleSubmit();
            }
          }}
        />
        <button
          onClick={handleSubmit}
          disabled={!input.trim() || isLoading}
          className={cn(
            "absolute right-2 top-1/2 -translate-y-1/2 p-2 rounded-lg transition-all",
            input.trim() && !isLoading
              ? "bg-emerald-600 hover:bg-emerald-700 text-white shadow"
              : "bg-stone-200 text-stone-400 cursor-not-allowed"
          )}
        >
          {isLoading ? (
            <Loader2 className="w-4 h-4 animate-spin" />
          ) : (
            <Send className="w-4 h-4" />
          )}
        </button>
      </div>
    </div>
  );
}
