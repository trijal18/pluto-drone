from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn
from plutocontrol import pluto
import asyncio

app = FastAPI()
drone = pluto()

html = """
<!DOCTYPE html>
<html>
<head>
    <title>Pluto Drone GCS</title>
</head>
<body>
    <h1>Ground Control Station</h1>
    <button onclick="sendCommand('arm')">Arm</button>
    <button onclick="sendCommand('disarm')">Disarm</button>
    <button onclick="sendCommand('takeoff')">Take Off</button>
    <button onclick="sendCommand('land')">Land</button>
    <p>Telemetry:</p>
    <pre id="telemetry"></pre>
    <script>
        let socket = new WebSocket("ws://localhost:8000/ws");

        socket.onmessage = function(event) {
            document.getElementById("telemetry").innerText = event.data;
        };

        function sendCommand(cmd) {
            socket.send(cmd);
        }
    </script>
</body>
</html>
"""

@app.get("/")
async def get():
    return HTMLResponse(html)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    drone.connect()
    time.sleep(3)
    try:
        while True:
            data = await websocket.receive_text()
            if data == "arm":
                drone.arm()
            elif data == "disarm":
                drone.disarm()
            elif data == "takeoff":
                drone.take_off()
            elif data == "land":
                drone.land()

            # Send telemetry
            telemetry = {
                "roll": drone.get_roll(),
                "pitch": drone.get_pitch(),
                "yaw": drone.get_yaw(),
                "height": drone.get_height(),
                "battery": drone.get_battery()
            }
            await websocket.send_text(str(telemetry))
            await asyncio.sleep(1)

    except WebSocketDisconnect:
        print("Client disconnected")
        drone.disconnect()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
