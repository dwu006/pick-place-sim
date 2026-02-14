# Pick-and-Place Mini Store — Pivot Design

**Date:** 2026-02-12  
**Context:** LabLab AI "Launch and Fund Your Own Startup – Edition 1" (AI Meets Robotics). Pivot from latte art to simulation-first pick-and-place mini store.

## Product Summary

**Mini Store:** User says what they want in natural language. Gemini parses the request into a structured pick list. A simulated robot (hardcoded IK, no learning) picks items from inventory and places them in front of the user. Backend runs on **Vultr** as the central system of record; **Gemini** powers NL understanding and planning. Fits **Track 3: Robotic Interaction and Task Execution (Simulation-First)** and satisfies **Vultr** (backend on Vultr) and **Gemini** (AI for reasoning/planning) sponsor requirements.

## Architecture

- **User** → Natural language ("I want two apples and a water")  
- **Frontend** → POST /api/orders → **Backend (Vultr)**  
- **Backend** → Gemini: NL → structured order (item IDs + quantities)  
- **Backend** → Worker runs "simulation": hardcoded IK / scripted pick-place steps (no physics engine required for MVP)  
- **WebSocket** → Real-time status: `planning` → `picking` (with step updates) → `completed`  
- **Frontend** → Order list + active order view with robot step log or simple 2D demo

## Data Model

- **Order**
  - `id`, `natural_language_input`, `pick_list` (JSON: `[{ "item_id": str, "quantity": int }]`), `status`, `error_message`, `created_at`, `updated_at`
- **Store inventory** (fixed): list of `item_id` and display names (e.g. apple, water, banana, chips). Stored in config or code; no DB table for MVP.

## Pipeline (Worker)

1. **Planning** — Call Gemini to parse `natural_language_input` → `pick_list` (items from fixed inventory). Validate quantities and item IDs.
2. **Picking** — Run mock "IK" simulator: for each (item, qty) emit WebSocket events (e.g. "moving_to_pick", "picking", "placing") with small delays. No real physics; demonstrative only.
3. **Completed** — Set `status = completed`. Optionally store a short log of steps in Order or keep only in WebSocket for demo.

## Tech Stack

| Layer        | Choice |
|-------------|--------|
| Backend     | FastAPI, SQLite (async), WebSocket (existing pattern) |
| NL / Planning | Gemini API (e.g. Gemini 2.0 Flash) — structured JSON output |
| Simulation  | In-process Python: scripted steps + asyncio.sleep; optional 2D canvas on frontend for visualization |
| Frontend    | Next.js 15, Tailwind; mini store UI: "What do you want?" + order list + active order + step feed |
| Deployment  | Backend and (optionally) frontend on **Vultr** VM; env for `GEMINI_API_KEY` |

## What We Remove

- All latte-art domain: `PourSpec`, patterns, evaluator, fluid_simulator, FluidLab references.
- `Job` and `TrajectoryCache` models replaced by `Order` and optional step log.

## What We Reuse

- FastAPI app, CORS, lifespan, static files.
- SQLAlchemy async + SQLite.
- WebSocket broadcast per "job" (now per order).
- Background worker pattern (poll for `queued` orders).
- Next.js app, Tailwind, React Query, WebSocket hook (adapted to orders).

## Out of Scope for MVP

- Real Isaac Sim / Gazebo integration (can be Phase 2).
- Real IK solver (use named steps like "pick_apple", "place_at_delivery" only).
- User accounts, payments, real inventory DB.
- Physical robots.

## Success Criteria

- User can submit a natural language order and see it turn into a pick list.
- Order status progresses planning → picking → completed with live WebSocket updates.
- Backend deployable on Vultr with public URL; frontend can point to it.
- Demo video and README explain: NL → Gemini → Vultr backend → simulated pick-place.
