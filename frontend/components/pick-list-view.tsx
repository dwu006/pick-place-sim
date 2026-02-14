"use client";

import { PickListItem } from "@/lib/types";

interface PickListViewProps {
  pickList: PickListItem[] | null;
}

const itemNames: Record<string, string> = {
  red_block: "Red block",
  blue_block: "Blue block",
  green_block: "Green block",
  cup: "Cup",
  bottle: "Bottle",
  toy: "Toy",
  book: "Book",
  box: "Box",
};

function nameFor(id: string): string {
  return itemNames[id] ?? id;
}

export function PickListView({ pickList }: PickListViewProps) {
  if (!pickList || pickList.length === 0) {
    return (
      <div className="text-sm text-stone-400 italic">No objects parsed yet.</div>
    );
  }

  return (
    <ul className="space-y-2">
      {pickList.map((item, i) => (
        <li
          key={`${item.item_id}-${i}`}
          className="flex items-center justify-between text-sm py-1.5 border-b border-stone-100 last:border-0"
        >
          <span className="text-stone-800 font-medium">
            {nameFor(item.item_id)}
          </span>
          <span className="text-stone-500">× {item.quantity}</span>
        </li>
      ))}
    </ul>
  );
}
