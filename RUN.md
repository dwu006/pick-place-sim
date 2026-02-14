# How to run stuff

From repo root: `c:\Users\happy\Documents\Coding\pick-place-sim`

---

## 1. Backend (API + WebSocket)

```bash
cd backend
python -m venv venv
# Windows:
venv\Scripts\activate
# Then:
pip install -r requirements.txt
# Add .env with GEMINI_API_KEY=...
uvicorn main:app --reload --port 8000
```

API: http://localhost:8000  
Docs: http://localhost:8000/docs

---

## 2. Frontend (web UI)

```bash
cd frontend
npm install
npm run dev
```

App: http://localhost:3000  
Proxies `/api` and `/ws` to backend:8000 when running locally.

---

## 3. MuJoCo scene viewer (Franka + box + objects)

Just the sim, no backend:

```bash
# From repo root
python backend/view_scene.py
```

Requires: `mujoco`, `mujoco_menagerie` (clone next to repo or set `MUJOCO_MENAGERIE_PATH`), optional `object_sim` for mesh objects.

---

## 4. Sim client (watch robot execute an order)

Backend must be running. Then:

```bash
# From repo root (or from backend with backend on path)
python backend/sim_client.py <order_id>
# Or point at remote backend:
python backend/sim_client.py <order_id> --backend-url ws://YOUR_IP:8000
```

Create an order from the frontend first, copy the `order_id`, then run this to see the Franka pick/place in MuJoCo.

---

## 5. Preview one object_sim mesh

```bash
# From repo root — list objects
python -m backend.sim.preview_object_sim

# Open viewer for one object (e.g. cup)
python -m backend.sim.preview_object_sim cup
```

Requires: `object_sim` at repo root, `mujoco`.

---

## Quick start (full stack)

1. Terminal 1: `cd backend` → activate venv → `uvicorn main:app --reload --port 8000`
2. Terminal 2: `cd frontend` → `npm run dev`
3. Browser: http://localhost:3000 — submit a task, copy the order ID
4. Terminal 3: `python backend/sim_client.py <order_id>` — watch the robot in MuJoCo
