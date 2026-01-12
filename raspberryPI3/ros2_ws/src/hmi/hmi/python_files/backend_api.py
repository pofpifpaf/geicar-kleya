from fastapi import FastAPI
from pydantic import BaseModel
from typing import Dict

app = FastAPI(title="HMI Backend API")

# ====== STATE GLOBAL (mémoire) ======
telemetry_state: Dict = {
    "speed": 0.0,
    "RPM": 0.0,
    "battery": 0.0,
    "pressure": 0.0,
    "temperature": 0.0,
    "airbag_state": "None",
    "collision_state": "None",
    "esp_state": "None",
}

adas_state: Dict = {
    "Collision": True,
    "ESP": True,
    "Airbag": True,
}

# ====== MODELS ======
class Telemetry(BaseModel):
    speed: float
    RPM: float
    battery: float
    pressure: float
    temperature: float
    airbag_state: str
    collision_state: str
    esp_state: str

class AdasConfig(BaseModel):
    Collision: bool
    ESP: bool
    Airbag: bool


# ====== TELEMETRY ======
@app.post("/telemetry")
def update_telemetry(data: Telemetry):
    telemetry_state.update(data.dict())
    return {"status": "ok"}

@app.get("/state")
def get_state():
    return telemetry_state


# ====== ADAS ======
@app.get("/adas")
def get_adas():
    return adas_state

@app.post("/adas")
def update_adas(cfg: AdasConfig):
    adas_state.update(cfg.dict())
    return {"status": "ok"}
