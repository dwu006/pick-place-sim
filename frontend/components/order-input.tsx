"use client";

import { useState } from "react";
import { Send, Loader2, Wand2 } from "lucide-react";
import { cn } from "@/lib/utils";

interface OrderInputProps {
  onSubmit: (input: string) => void;
  isLoading: boolean;
}

const suggestions = [
  "Pick up the duck",
  "Clean up all the toys",
  "Put the phone in the bin",
  "Tidy up the banana and elephant",
];

export function OrderInput({ onSubmit, isLoading }: OrderInputProps) {
  const [input, setInput] = useState("");

  const handleSubmit = () => {
    if (!input.trim() || isLoading) return;
    onSubmit(input.trim());
    setInput("");
  };

  return (
    <div className="space-y-4">
      <label className="text-sm font-medium text-slate-300 flex items-center gap-2">
        <Wand2 className="w-4 h-4 text-amber-400" />
        What should the robot clean up?
      </label>
      <div className="relative">
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder='e.g. "Pick up the duck and banana"'
          className={cn(
            "w-full px-4 py-3.5 pr-14 rounded-xl text-white placeholder:text-slate-500",
            "focus:outline-none focus:ring-2 focus:ring-emerald-500/50 transition-all",
            "bg-slate-700/50 border border-white/10"
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
            "absolute right-2 top-1/2 -translate-y-1/2 p-2.5 rounded-lg transition-all",
            input.trim() && !isLoading
              ? "bg-gradient-to-r from-emerald-500 to-cyan-500 hover:from-emerald-400 hover:to-cyan-400 text-white shadow-lg shadow-emerald-500/25"
              : "bg-slate-600 text-slate-400 cursor-not-allowed"
          )}
        >
          {isLoading ? (
            <Loader2 className="w-4 h-4 animate-spin" />
          ) : (
            <Send className="w-4 h-4" />
          )}
        </button>
      </div>
      <div className="flex flex-wrap gap-2">
        {suggestions.map((s) => (
          <button
            key={s}
            onClick={() => setInput(s)}
            className="text-xs px-3 py-1.5 rounded-full bg-slate-700/50 text-slate-400 hover:text-white hover:bg-slate-600/50 border border-white/5 transition-all"
          >
            {s}
          </button>
        ))}
      </div>
    </div>
  );
}
