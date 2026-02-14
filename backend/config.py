from typing import List

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    cors_origins: List[str] = ["http://localhost:3000"]
    database_url: str = "sqlite+aiosqlite:///./ministore.db"
    gemini_api_key: str = ""
    gemini_model: str = "gemini-3-flash-preview"
    # Gemini Robotics-ER 1.5 for vision-language / robotics (e.g. gemini-robotics-er-1.5-preview).
    gemini_robotics_model: str = "gemini-robotics-er-1.5-preview"
    # Set to false to run without SQLite (in-memory orders only). Good for demos; Vultr submission should use True.
    use_database: bool = True
    # When True, Gemini generates the full pick-place plan upfront (plan-first, faster than function calling). When False, use hardcoded sequence.
    use_gemini_robot_agent: bool = True
    # When True, spawning the MuJoCo sim client when an order is created (local dev only; set SPAWN_SIM_CLIENT=true in .env).
    spawn_sim_client: bool = False

    class Config:
        env_file = ".env"


settings = Settings()
