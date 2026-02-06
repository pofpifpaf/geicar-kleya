from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import asyncio
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
    "lca_state": "None",
}

adas_state: Dict = {
    "collision": False,
    "esp": False,
    "airbag": False,
    "lca": False,
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
    lca_state: str

class AdasConfig(BaseModel):
    collision: bool
    esp: bool
    airbag: bool
    lca: bool


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


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.websocket("/ws/state")
async def websocket_state(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(telemetry_state)
            await asyncio.sleep(0.2)
    except (WebSocketDisconnect, Exception):
        # Le client a fermé l'onglet / perdu le réseau / reload la page
        # => on sort proprement sans stacktrace
        return
