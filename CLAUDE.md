# LatteBot Latte Art Simulation System

## Project Overview
LatteBot is a latte art simulation and training system that combines:
- Natural language pour planning using Gemini AI
- Differentiable fluid simulation using FluidLab
- Automated quality evaluation and feedback
- Full-stack web dashboard for interactive control

## Architecture

### System Components
1. **Backend (FastAPI)**: RESTful API with WebSocket support for real-time updates
2. **Frontend (Next.js)**: Interactive dashboard for job creation and monitoring
3. **FluidLab**: Differentiable physics engine for fluid simulation (Taichi-based)
4. **Gemini AI Integration**: Natural language processing for planning and evaluation

### Pipeline Flow
```
User Input (NL) → Gemini Planner → Pour Spec → FluidLab Sim → Video → Gemini Evaluator → Score & Feedback
```

## Completed Work

### 1. Backend Infrastructure (FastAPI)
- [x] FastAPI application setup with CORS middleware
- [x] SQLAlchemy async ORM with SQLite database
- [x] Database models: `Job` and `TrajectoryCache`
- [x] Pydantic schemas: `PourSpec`, `JobResponse`, `EvaluationResult`
- [x] Background worker for job processing
- [x] WebSocket support for real-time job status updates

**Key Files:**
- `backend/main.py` - FastAPI app initialization
- `backend/worker.py` - Background job processor
- `backend/models.py` - Database schema
- `backend/schemas.py` - Pydantic models
- `backend/database.py` - Database connection
- `backend/config.py` - Configuration settings

### 2. API Endpoints
- [x] `POST /api/jobs` - Create new latte art job
- [x] `GET /api/jobs` - List all jobs
- [x] `GET /api/jobs/{job_id}` - Get job details
- [x] `GET /api/patterns` - List pattern presets
- [x] `WS /ws/{job_id}` - WebSocket for real-time updates

**Router Files:**
- `backend/routers/jobs.py`
- `backend/routers/patterns.py`
- `backend/routers/websocket.py`

### 3. Gemini AI Integration
- [x] **Planner Service** (`gemini_planner.py`):
  - Converts natural language descriptions to pour parameters
  - Uses Gemini 2.0 Flash with structured JSON output
  - Schema validation for pour specifications
  - Fallback defaults for rosetta, tulip, heart patterns

- [x] **Evaluator Service** (`gemini_evaluator.py`):
  - Evaluates pour quality based on parameters
  - Provides scores (0-100) and detailed feedback
  - Breakdown scores: contrast, symmetry, definition, crema_quality
  - Intelligent heuristics for pattern-specific evaluation

### 4. Fluid Simulation (FluidLab Integration)
- [x] FluidLab repository added to project
- [x] Mock simulator service (`fluid_simulator.py`) created
- [ ] **TODO**: Replace mock with real FluidLab integration

**FluidLab Features:**
- Differentiable physics engine (Taichi-based)
- Multi-material support (liquids, solids, gases)
- Two rendering modes: GGUIRenderer (fast) and GLRenderer (high-quality)
- Trajectory optimization via differentiable physics

### 5. Frontend Dashboard (Next.js)
- [x] Next.js 15 application initialized
- [x] TypeScript configuration
- [x] TailwindCSS styling setup
- [x] Basic UI layout structure
- [ ] **TODO**: Implement full dashboard UI components

**Frontend Structure:**
- `frontend/app/page.tsx` - Main page component
- `frontend/app/layout.tsx` - Root layout
- `frontend/app/globals.css` - Global styles

## TODO List

### High Priority

#### 1. FluidLab Integration
- [ ] Set up FluidLab Python environment (conda)
  ```bash
  cd FluidLab/
  conda env create -f environment.yml
  conda activate fluidlab
  pip install -e .
  ```
- [ ] Create Python wrapper for FluidLab latte art simulation
- [ ] Implement pour trajectory generation from `PourSpec` parameters
- [ ] Configure rendering pipeline (start with GGUIRenderer for speed)
- [ ] Save simulation output as video files to `backend/static/videos/`
- [ ] Update `fluid_simulator.py` to call real FluidLab engine
- [ ] Handle simulation errors and timeouts gracefully

#### 2. Frontend Dashboard Implementation
- [ ] **Job Creation UI**:
  - Natural language input form
  - Pattern preset selection buttons (rosetta, tulip, heart)
  - Submit button to create job

- [ ] **Job Monitoring UI**:
  - Job list with status indicators
  - Real-time status updates via WebSocket
  - Progress indicators for planning/simulating/evaluating stages

- [ ] **Results Display**:
  - Video player for simulation results
  - Score display with breakdown visualization
  - Feedback text presentation
  - Parameter details panel

- [ ] **API Integration**:
  - Fetch jobs from `/api/jobs`
  - Create jobs via `/api/jobs`
  - WebSocket connection for live updates
  - Error handling and loading states

#### 3. Video Storage & Serving
- [ ] Create `backend/static/videos/` directory
- [ ] Implement video file naming convention (job_id-based)
- [ ] Configure static file serving for video playback
- [ ] Add video cleanup/archival strategy for old jobs

#### 4. Enhanced Gemini Evaluation
- [ ] Update evaluator to accept actual simulation images/video
- [ ] Use Gemini's multimodal capabilities for visual analysis
- [ ] Compare target pattern with achieved result
- [ ] Provide specific visual feedback (e.g., "top leaf is too narrow")

### Medium Priority

#### 5. Trajectory Optimization
- [ ] Implement trajectory caching using `TrajectoryCache` model
- [ ] Add hash function for `PourSpec` objects
- [ ] Cache successful pour trajectories for reuse
- [ ] Add endpoint to retrieve cached trajectories

#### 6. Robot Control Integration
- [ ] Design robot action space mapping from pour parameters
- [ ] Implement trajectory-to-robot-commands converter
- [ ] Add Isaac Sim integration for robot visualization
- [ ] Create dual Franka robot coordination system

#### 7. Testing & Validation
- [ ] Unit tests for Gemini planner/evaluator services
- [ ] Integration tests for full pipeline
- [ ] End-to-end tests with real FluidLab simulations
- [ ] Frontend component tests
- [ ] Load testing for concurrent job processing

#### 8. Documentation
- [ ] API documentation (OpenAPI/Swagger)
- [ ] Frontend component documentation
- [ ] FluidLab integration guide
- [ ] Deployment instructions
- [ ] User guide for dashboard

### Low Priority

#### 9. Advanced Features
- [ ] Custom pattern upload/creation
- [ ] Pattern library with search
- [ ] Historical job analytics
- [ ] Comparative analysis between attempts
- [ ] User accounts and authentication
- [ ] Sharing pour recipes/results

#### 10. Performance Optimization
- [ ] Implement job queue with priority levels
- [ ] Add GPU queue management for simulations
- [ ] Optimize database queries with proper indexing
- [ ] Add Redis for caching and job queue
- [ ] Implement rate limiting for API

#### 11. DevOps
- [ ] Docker containerization
- [ ] Docker Compose for full stack
- [ ] CI/CD pipeline setup
- [ ] Automated testing in CI
- [ ] Production deployment configuration

## Current Status

### Working
- Backend API server with async job processing
- Gemini-based natural language planning
- Gemini-based quality evaluation
- Database persistence for jobs
- WebSocket real-time updates
- Mock simulation pipeline

### In Progress
- FluidLab integration (repository added, needs Python wrapper)
- Frontend dashboard (initialized, needs UI implementation)

### Blocked/Pending
- Real simulation videos (depends on FluidLab integration)
- Visual quality evaluation (depends on simulation videos)
- Robot control (depends on simulation validation)

## Development Setup

### Backend
```bash
cd backend/
python -m venv venv
source venv/bin/activate  # or `venv\Scripts\activate` on Windows
pip install -r requirements.txt
# Create .env file with GEMINI_API_KEY
uvicorn main:app --reload --port 8000
```

### Frontend
```bash
cd frontend/
npm install
npm run dev  # runs on port 3000
```

### FluidLab
```bash
cd FluidLab/
conda env create -f environment.yml
conda activate fluidlab
pip install -e .
```

## Key Design Decisions

1. **Async Processing**: Jobs run in background worker to avoid blocking API
2. **WebSockets**: Real-time updates for better UX during long simulations
3. **Gemini AI**: Leverages multimodal LLM for both planning and evaluation
4. **FluidLab**: Differentiable physics enables future gradient-based optimization
5. **Mock-First**: Mock simulator allows end-to-end testing before FluidLab integration

## References

- [FluidLab Paper (ICLR 2023)](https://arxiv.org/abs/2303.02346)
- [FluidLab Project Page](https://fluidlab2023.github.io/)
- [Lattebot Architecture PDF](./Lattebot%20Architecture.pdf)
