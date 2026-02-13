from typing import List

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    cors_origins: List[str] = ["http://localhost:3000"]
    database_url: str = "sqlite+aiosqlite:///./ministore.db"
    gemini_api_key: str = ""
    # Set to false to run without SQLite (in-memory orders only). Good for demos; Vultr submission should use True.
    use_database: bool = True
    # When True, Gemini generates the full pick-place plan upfront (plan-first, faster than function calling). When False, use hardcoded sequence.
    use_gemini_robot_agent: bool = True

    class Config:
        env_file = ".env"


settings = Settings()
