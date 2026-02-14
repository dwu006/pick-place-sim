# Cleanup Room — Pick & Place (AI Meets Robotics)

## Project Overview

Cleanup Room is a **simulation-first pick-and-place** demo for the LabLab AI "Launch and Fund Your Own Startup – Edition 1" hackathon (AI Meets Robotics). The user says what to tidy in natural language; **Gemini** parses the request into a structured pick list; a **simulated robot** (hardcoded IK-style steps, no learning) picks objects from the room and places them in a bin. The **backend runs on Vultr** as the central system of record. The **MuJoCo sim** includes a room (floor, walls, table with scatter objects, bin).

- **Track**: Robotic Interaction and Task Execution (Simulation-First)
- **Sponsor tech**: Gemini (NL understanding, planning), Vultr (mandatory backend)

## Architecture

### Pipeline

```
User: "Pick up the red block and the cup"
  → Frontend POST /api/orders
  → Backend (Vultr) enqueues order, broadcasts "new_order" on /ws/sim
  → Sim client (if running): receives new_order → renders hand_camera frame → POSTs PNG to /api/orders/{id}/wrist_image
  → Backend: stores wrist image in memory (keyed by order_id)
  → Worker: get_wrist_image(order_id); parse_order(nl, image_bytes=…)
     → If image present: Gemini Robotics (ER 1.5) with image + text → pick_list
     → Else or on failure: text-only Gemini → pick_list
  → Worker: Simulated pick-place (move_to_pick → pick → move_to_delivery → place) per object → bin
  → WebSocket: Real-time status + robot steps
  → Frontend: Task list, objects to tidy, live robot step log
```

### Components

1. **Backend (FastAPI)** — Central system of record. Runs on **Vultr**.
   - SQLite (async), REST API, WebSocket for live updates
   - Gemini: natural language → structured cleanup list (pick list)
   - In-process "simulator": scripted pick/place steps (no physics engine)

2. **Frontend (Next.js)** — Web UI for cleanup tasks and monitoring.
   - "What should the robot clean up?" input, task list, objects to tidy, robot step log
   - Optional: deploy on Vultr (same VM) or Vercel with `NEXT_PUBLIC_API_BASE` pointing to Vultr backend

3. **Room objects** — Fixed list in code (e.g. red_block, blue_block, green_block, cup, bottle, toy, book, box). Gemini maps user phrases to these `item_id`s. Robot places them in the bin.

## Key Files

| Area        | Files |
|------------|--------|
| Backend    | `backend/main.py`, `backend/worker.py`, `backend/models.py`, `backend/schemas.py`, `backend/routers/orders.py`, `backend/routers/websocket.py` |
| Services   | `backend/services/gemini_order_parser.py` (text + vision), `backend/services/gemini_robotics.py` (ER 1.5), `backend/services/gemini_planner.py` (plan generation), `backend/services/gemini_robot_agent.py` (plan execution), `backend/services/pick_place_simulator.py` (fallback), `backend/order_store.py` (orders + wrist images) |
| Sim client | `backend/sim_client.py` — MuJoCo viewer, connects to `/ws/sim`, captures `hand_camera`, POSTs wrist image, receives steps via `/ws/{order_id}`. Run: `python sim_client.py --wait --backend-url ws://localhost:8000`. |
| Sim        | `backend/sim/` — MuJoCo Franka scene with room (floor, walls, table, scatter objects, bin; optional; requires mujoco_menagerie). Optional: **object_sim** ([vikashplus/object_sim](https://github.com/vikashplus/object_sim)) for richer object meshes — clone into repo root; see `get_object_sim_path()`, `OBJECT_SIM_OBJECT_NAMES`, `backend/sim/preview_object_sim.py`. |
| Frontend   | `frontend/app/page.tsx`, `frontend/components/order-input.tsx`, `frontend/components/order-list.tsx`, `frontend/components/robot-step-log.tsx`, `frontend/components/pick-list-view.tsx` |
| Config     | `backend/config.py` (DB path, CORS, `GEMINI_API_KEY`) |

## API

- `POST /api/orders` — Body: `{ "natural_language_input": "..." }`. Creates order, returns `OrderResponse`.
- `GET /api/orders` — List recent orders.
- `GET /api/orders/{order_id}` — Get one order.
- `POST /api/orders/{order_id}/wrist_image` — Body: raw PNG bytes (`Content-Type: image/png`). Sim client uploads wrist-camera frame for vision-based parsing.
- `GET /api/store/items` — List store inventory (id, name, description).
- `WS /ws/sim` — Backend sends `new_order` { order_id }; sim client subscribes and uploads wrist image, then connects to `/ws/{order_id}` for steps.
- `WS /ws/{order_id}` — Real-time: `status_update`, `pick_list_ready`, `robot_step`, `order_complete`, `error`.

## Development

### Backend

```bash
cd backend/
python -m venv venv
# Windows: venv\Scripts\activate
pip install -r requirements.txt
# .env: GEMINI_API_KEY=...
uvicorn main:app --reload --port 8000
```

### Frontend

```bash
cd frontend/
npm install
npm run dev   # port 3000; rewrites proxy /api and /ws to backend:8000
```

### Env

- **Backend**: `GEMINI_API_KEY` (required). Optional: `USE_DATABASE=false` (in-memory orders only); `USE_GEMINI_ROBOT_AGENT=true` (default) so Gemini generates the full plan upfront (plan-first approach).
- **Frontend**: `NEXT_PUBLIC_API_BASE` optional; if set (e.g. `https://your-backend.vultr.com`), API and WebSocket use that host.

## Deploying on Vultr (Hackathon Requirement)

Vultr must host the **backend** as the central system of record.

1. **Create a Vultr VM** (e.g. Ubuntu 22.04). Open ports: 80, 443, 8000 (or reverse-proxy 80→8000).

2. **On the VM** — Backend only (minimal):
   ```bash
   sudo apt update && sudo apt install -y python3.11-venv
   git clone <your-repo> && cd latte-art-sim/backend
   python3 -m venv venv && source venv/bin/activate
   pip install -r requirements.txt
   # Set GEMINI_API_KEY in .env or export
   uvicorn main:app --host 0.0.0.0 --port 8000
   ```
   Or run under gunicorn + systemd for production.

3. **CORS**: In `backend/config.py`, set `cors_origins` to include your frontend origin (e.g. `https://your-frontend.vercel.app` or `https://your-vultr-ip` if you serve the frontend on the same VM).

4. **Frontend pointing to Vultr**: Build with:
   ```bash
   NEXT_PUBLIC_API_BASE=https://your-backend.vultr.com npm run build
   ```
   Then deploy the frontend anywhere (Vercel, or same Vultr VM with `npm run start` behind nginx). Users will hit the frontend; the frontend will call and open WebSockets to `NEXT_PUBLIC_API_BASE`.

5. **Optional — All on one Vultr VM**: Serve FastAPI on 8000 and build/serve Next.js (e.g. port 3000), then put nginx in front: `/` → Next.js, `/api` and `/ws` → FastAPI. Then no `NEXT_PUBLIC_API_BASE` needed.

## Why a database?

The app can run **without a database** (`USE_DATABASE=false`). Orders are then kept in memory; the backend still queues and processes them, and the frontend works the same. Use this for quick demos or when you don’t want to manage SQLite. For the hackathon, Vultr expects a “central system of record” — so for submission, keep `USE_DATABASE=true` (default) so orders are persisted.

## MuJoCo + Franka FR3 (optional)

To show the Franka arm in simulation:

1. **Clone [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)** (Franka FR3 requires MuJoCo 3.1.3+):
   ```bash
   git clone https://github.com/google-deepmind/mujoco_menagerie
   # Put it next to the repo or set MUJOCO_MENAGERIE_PATH to its path
   ```
2. **Install MuJoCo**: `pip install mujoco>=3.1.3`
3. **Scene with table**: `backend/sim/` loads the menagerie’s `franka_fr3/scene.xml`, injects a table under the robot, and exposes `load_scene_with_table()` → `(model, data)`. Use this for a viewer or for feeding poses into the same pipeline later.

## object_sim (optional)

[object_sim](https://github.com/vikashplus/object_sim) provides MuJoCo models for daily objects (cup, mug, waterbottle, cubes, apple, rubberduck, etc.). Clone into the repo root: `git clone https://github.com/vikashplus/object_sim.git`. **Preview with real meshes:** object_sim includes `preview.py` that loads each object via `from_xml_path()` so mesh paths resolve. From repo root run: `python run_object_sim_preview.py -o cup` (one object) or `python run_object_sim_preview.py` (cycle all; requires `click`). Lightweight single-object viewer: `python -m backend.sim.preview_object_sim cup`. The main room scene uses primitives (no STL) so it always renders; use the preview scripts to see object_sim meshes.

## Gemini doing the work (plan-first approach)

By default, **Gemini generates the full pick-place plan upfront** (plan-first, better than function calling). For each task, Gemini receives the pick list and outputs a structured plan: `{"steps": [{"step": "move_to_pick", "item_id": "red_block", "message": "..."}, ...], "reasoning": "..."}`. The backend then executes this plan step-by-step, broadcasting each action. This approach:
- **Single API call** (faster, cheaper than function calling)
- **Shows Gemini's reasoning** (the "reasoning" field explains the strategy)
- **Deterministic execution** (plan is generated once, then executed reliably)
- **Still demonstrates AI planning** (Gemini decides the sequence and repetition)

See `backend/services/gemini_planner.py` (plan generation) and `backend/services/gemini_robot_agent.py` (plan execution). To fall back to the hardcoded step sequence, set `USE_GEMINI_ROBOT_AGENT=false`.

## Wrist camera & Gemini Robotics (ER 1.5)

**What’s done:**

- **Sim client** (`backend/sim_client.py`): On `new_order`, renders one frame from `hand_camera` (MuJoCo `Renderer` 640×480), encodes PNG, POSTs to `POST /api/orders/{order_id}/wrist_image`. Requires `httpx` and `Pillow` in `backend/requirements.txt`.
- **Backend**: `order_store.set_wrist_image` / `get_wrist_image` (in-memory); `POST /api/orders/{order_id}/wrist_image` stores image per order.
- **Parser** (`backend/services/gemini_order_parser.py`): `parse_order(nl_input, image_bytes=None)`. When `image_bytes` is present, calls `gemini_robotics.generate_robotics_json` (ER 1.5) with image + prompt for pick list (same schema: `ROOM_OBJECT_IDS`, JSON array of `{item_id, quantity}`); validates and returns list. On failure or no image, falls back to text-only Gemini.
- **Worker** (`backend/worker.py`): Gets `get_wrist_image(order_id)` before parsing and passes to `parse_order`. IK and planner unchanged; ER 1.5 only augments where the pick list comes from.

**What’s left (optional / later):**

- **Timing**: Sim client must POST the wrist image before the worker runs; if the worker picks the order first, parsing is text-only. Consider delaying worker slightly for new orders or having the worker wait for an image with a short timeout.
- **ER 1.5 function-calling**: Use high-level actions (e.g. `move(x,y)`, `setGripperState(open)`) from ER 1.5 and map normalized 2D to 3D for IK (documented in plan; not implemented).
- **2D→3D**: Use ER 1.5 object points/labels + table plane + camera model to derive 3D targets for IK (optional).
- **Second camera**: Fixed overhead or other viewpoint (optional).
- **Persistence**: Wrist images are in-memory only; no DB column or file store yet.

## Design Doc

- `docs/plans/2026-02-12-pick-place-mini-store-design.md` — Pivot design and scope.

## What Was Removed (Latte Art Pivot)

- Latte art domain: `Job`, `PourSpec`, patterns, evaluator, fluid_simulator, FluidLab references.
- Old frontend: pattern presets, simulation viewer, evaluation panel, pour spec viewer, job list (replaced by order equivalents).

## References

- [LabLab AI Hackathon](https://lablab.ai) — Launch and Fund Your Own Startup, Edition 1 (AI Meets Robotics).
- [Vultr](https://www.vultr.com) — Backend hosting (mandatory for hackathon).
- [Gemini API](https://ai.google.dev/) — NL order parsing.
