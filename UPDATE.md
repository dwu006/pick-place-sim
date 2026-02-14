# Update log

## Wrist camera + Gemini Robotics (ER 1.5) integration

**Summary:** Sim client captures one frame from the wrist camera when an order is created, uploads it to the backend, and the parser uses Gemini Robotics (ER 1.5) with image + text to produce the pick list when an image is available; otherwise it falls back to text-only parsing. IK and the rest of the pipeline are unchanged.

### Done

- **Sim client** (`backend/sim_client.py`)
  - On `new_order`: render one frame from `hand_camera` (MuJoCo `Renderer` 640×480), encode PNG (Pillow), POST to `POST /api/orders/{order_id}/wrist_image`.
  - Dependencies: `httpx`, `Pillow` in `backend/requirements.txt`.
- **Backend**
  - `order_store`: in-memory `_wrist_images`, `set_wrist_image(order_id, bytes)`, `get_wrist_image(order_id)`.
  - `POST /api/orders/{order_id}/wrist_image`: accept raw PNG body, store via `set_wrist_image`.
- **Worker** (`backend/worker.py`): Before parsing, `get_wrist_image(order_id)` and pass to `parse_order(nl_input, image_bytes=wrist_image)`.
- **Parser** (`backend/services/gemini_order_parser.py`): `parse_order(natural_language_input, image_bytes=None)`. When `image_bytes` is set, call `gemini_robotics.generate_robotics_json` (ER 1.5) with image + prompt for pick list (same schema: `ROOM_OBJECT_IDS`, JSON array of `{item_id, quantity}`); validate and return list. On failure or no image, use existing text-only Gemini path.

### Not done (optional / later)

- Timing: worker may run before sim client POSTs the image → text-only parse for that order. Option: short delay or wait-for-image with timeout.
- ER 1.5 function-calling (move/gripper) and 2D→3D for targets.
- Second camera (e.g. fixed overhead).
- Persisting wrist images (DB or file store).

### How to run

1. **Backend:** `cd backend && venv\Scripts\activate && uvicorn main:app --reload --port 8000`
2. **Frontend:** `cd frontend && npm run dev` → http://localhost:3000
3. **Sim client:** `cd backend && venv\Scripts\python.exe sim_client.py --wait --backend-url ws://localhost:8000`

Submit a task from the frontend; the sim client will capture the wrist view, upload it, and the parser will use vision when the image is available.
