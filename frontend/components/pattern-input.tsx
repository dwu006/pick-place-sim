"use client";

import { useState } from "react";
import { Coffee, Flower2, Heart, Send, Loader2 } from "lucide-react";
import { cn } from "@/lib/utils";

const PRESETS = [
  { id: "rosetta", name: "Rosetta", icon: Flower2, prompt: "Make a beautiful rosetta with high contrast" },
  { id: "tulip", name: "Tulip", icon: Coffee, prompt: "Pour a classic tulip with defined layers" },
  { id: "heart", name: "Heart", icon: Heart, prompt: "Create a clean heart shape" },
] as const;

interface PatternInputProps {
  onSubmit: (input: string, preset?: string) => void;
  isLoading: boolean;
}

export function PatternInput({ onSubmit, isLoading }: PatternInputProps) {
  const [input, setInput] = useState("");
  const [selectedPreset, setSelectedPreset] = useState<string | null>(null);

  const handlePreset = (preset: (typeof PRESETS)[number]) => {
    setSelectedPreset(preset.id);
    setInput(preset.prompt);
  };

  const handleSubmit = () => {
    if (!input.trim() || isLoading) return;
    onSubmit(input, selectedPreset || undefined);
    setInput("");
    setSelectedPreset(null);
  };

  return (
    <div className="space-y-4">
      <div className="flex gap-2">
        {PRESETS.map((preset) => {
          const Icon = preset.icon;
          return (
            <button
              key={preset.id}
              onClick={() => handlePreset(preset)}
              className={cn(
                "flex-1 flex flex-col items-center gap-1.5 p-3 rounded-xl border-2 transition-all text-sm font-medium",
                selectedPreset === preset.id
                  ? "border-amber-600 bg-amber-50 text-amber-800"
                  : "border-stone-200 hover:border-amber-300 hover:bg-stone-50 text-stone-600"
              )}
            >
              <Icon className="w-5 h-5" />
              {preset.name}
            </button>
          );
        })}
      </div>

      <div className="relative">
        <textarea
          value={input}
          onChange={(e) => {
            setInput(e.target.value);
            setSelectedPreset(null);
          }}
          placeholder='Describe your latte art... e.g. "Make a rosetta with lots of contrast"'
          className="w-full h-24 px-4 py-3 bg-stone-50 border-2 border-stone-200 rounded-xl resize-none text-sm text-stone-800 placeholder:text-stone-400 focus:outline-none focus:border-amber-500 focus:ring-2 focus:ring-amber-500/20 transition-all"
          onKeyDown={(e) => {
            if (e.key === "Enter" && !e.shiftKey) {
              e.preventDefault();
              handleSubmit();
            }
          }}
        />
        <button
          onClick={handleSubmit}
          disabled={!input.trim() || isLoading}
          className={cn(
            "absolute bottom-3 right-3 p-2 rounded-lg transition-all",
            input.trim() && !isLoading
              ? "bg-amber-600 hover:bg-amber-700 text-white shadow-md"
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
