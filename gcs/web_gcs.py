# web_gcs.py
import asyncio
from fastapi import FastAPI, WebSocket, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from plutocontrol import pluto

app = FastAPI()

# Middleware for CORS (frontend can be on different origin)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# File paths
templates = Jinja2Templates(directory="templates")
app.mount("/static", StaticFiles(directory="static"), name="static")

# Drone instance
dr = pluto()

@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/connect")
async def connect():
    dr.connect()
    return {"status": "connected"}

@app.post("/arm")
async def arm():
    dr.arm()
    return {"status": "armed"}

@app.post("/takeoff")
async def takeoff():
    dr.take_off()
    return {"status": "takeoff"}

@app.post("/land")
async def land():
    dr.land()
    return {"status": "landing"}

@app.post("/disarm")
async def disarm():
    dr.disarm()
    return {"status": "disarmed"}

@app.get("/test_roll")
def test_roll():
    return {"roll": dr.get_roll()}

@app.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            try:
                data = {
                    "roll": dr.get_roll() or 0,
                    "pitch": dr.get_pitch() or 0,
                    "yaw": dr.get_yaw() or 0,
                    "height": dr.get_height() or 0,
                    "battery": dr.get_battery() or 0,
                }
            except Exception as e:
                data = {"error": str(e)}
                print("Telemetry error:", e)
            await websocket.send_json(data)
            await asyncio.sleep(0.5)
    except Exception as e:
        print("WebSocket closed:", e)
        await websocket.close()

