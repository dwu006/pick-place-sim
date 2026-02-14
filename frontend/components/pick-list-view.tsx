"use client";

import { PickListItem } from "@/lib/types";
import { Package } from "lucide-react";

interface PickListViewProps {
  pickList: PickListItem[] | null;
}

const itemNames: Record<string, { name: string; emoji: string }> = {
  banana: { name: "Banana", emoji: "🍌" },
  duck: { name: "Rubber Duck", emoji: "🦆" },
  phone: { name: "Phone", emoji: "📱" },
  elephant: { name: "Elephant Toy", emoji: "🐘" },
  eyeglasses: { name: "Eyeglasses", emoji: "👓" },
  flute: { name: "Flute", emoji: "🎵" },
  gamecontroller: { name: "Game Controller", emoji: "🎮" },
  headphones: { name: "Headphones", emoji: "🎧" },
  mouse: { name: "Computer Mouse", emoji: "🖱️" },
  piggybank: { name: "Piggy Bank", emoji: "🐷" },
  pyramidlarge: { name: "Pyramid Toy", emoji: "🔺" },
  stanfordbunny: { name: "Bunny Toy", emoji: "🐰" },
  train: { name: "Toy Train", emoji: "🚂" },
  watch: { name: "Watch", emoji: "⌚" },
  airplane: { name: "Toy Airplane", emoji: "✈️" },
  alarmclock: { name: "Alarm Clock", emoji: "⏰" },
  camera: { name: "Camera", emoji: "📷" },
};

function getItem(id: string): { name: string; emoji: string } {
  return itemNames[id] ?? { name: id, emoji: "📦" };
}

export function PickListView({ pickList }: PickListViewProps) {
  if (!pickList || pickList.length === 0) {
    return (
      <div className="text-sm text-slate-500 italic py-4 flex items-center gap-2">
        <Package className="w-4 h-4" />
        No objects parsed yet.
      </div>
    );
  }

  return (
    <ul className="space-y-2">
      {pickList.map((item, i) => {
        const { name, emoji } = getItem(item.item_id);
        return (
          <li
            key={`${item.item_id}-${i}`}
            className="flex items-center gap-3 px-3 py-2.5 rounded-xl bg-slate-700/30 border border-white/5"
          >
            <span className="text-xl">{emoji}</span>
            <span className="text-sm text-white font-medium flex-1">{name}</span>
            <span className="text-xs text-slate-400 bg-slate-600/50 px-2 py-0.5 rounded-full">
              x{item.quantity}
            </span>
          </li>
        );
      })}
    </ul>
  );
}
