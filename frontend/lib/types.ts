export type OrderStatus =
  | "queued"
  | "planning"
  | "picking"
  | "completed"
  | "failed";

export interface PickListItem {
  item_id: string;
  quantity: number;
}

export interface Order {
  id: string;
  natural_language_input: string;
  pick_list: PickListItem[] | null;
  status: OrderStatus;
  error_message: string | null;
  created_at: string;
  updated_at: string;
}

export interface StoreItem {
  id: string;
  name: string;
  description: string;
}

export interface RobotStep {
  type: "robot_step";
  step: string;
  item_id?: string;
  quantity_index?: number;
  message: string;
}
