from fastapi import FastAPI, WebSocket, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
import asyncio
from plutocontrol import pluto  # your drone control class

app = FastAPI()
dr = pluto()

# Static files & templates
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

@app.get("/", response_class=HTMLResponse)
async def home(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

# --- Control Endpoints ---
@app.post("/connect")
def connect(): dr.connect(); return {"status": "connected"}

@app.post("/disconnect")
def disconnect(): dr.disconnect(); return {"status": "disconnected"}

@app.post("/arm")
def arm(): dr.arm(); return {"status": "armed"}

@app.post("/disarm")
def disarm(): dr.disarm(); return {"status": "disarmed"}

@app.post("/takeoff")
def takeoff(): dr.take_off(); return {"status": "takeoff"}

@app.post("/land")
def land(): dr.land(); return {"status": "landing"}

@app.post("/forward")
def forward(): dr.forward(); return {"status": "forward"}

@app.post("/backward")
def backward(): dr.backward(); return {"status": "backward"}

@app.post("/left")
def left(): dr.left(); return {"status": "left"}

@app.post("/right")
def right(): dr.right(); return {"status": "right"}

# --- WebSocket Telemetry ---
@app.websocket("/ws/telemetry")
async def telemetry(websocket: WebSocket):
    await websocket.accept()
    while True:
        try:
            data = {
                "roll": dr.get_roll() or 0,
                "pitch": dr.get_pitch() or 0,
                "yaw": dr.get_yaw() or 0,
                "height": dr.get_height() or 0,
                "battery": dr.get_battery() or 0,
                "rc": dr.rc_values()
            }
            await websocket.send_json(data)
        except Exception as e:
            await websocket.send_json({"error": str(e)})
        await asyncio.sleep(0.5)
