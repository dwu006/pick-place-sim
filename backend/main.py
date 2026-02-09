from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def root():
    return {"message": "Latte Art Sim backend is alive ☕"}

