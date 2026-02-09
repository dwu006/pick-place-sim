from typing import List

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    cors_origins: List[str] = ["http://localhost:3000"]
    database_url: str = "sqlite+aiosqlite:///./latte_art.db"
    gemini_api_key: str = ""

    class Config:
        env_file = ".env"


settings = Settings()
