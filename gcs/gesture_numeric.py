from ultralytics import YOLO
import requests
import time
import logging

# === CONFIG ===
MODEL_PATH = "models/numeric.pt"
GCS_BASE_URL = "http://127.0.0.1:8000"
COOLDOWN_SECONDS = 3

gesture_to_command = {
    "1": "arm",
    "2": "takeoff",
    "3": "land",
    "4": "disarm"
}

# === Load Model ===
model = YOLO(MODEL_PATH)
last_gesture = None
last_time = 0

# === Setup Logging ===
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")

def send_command(cmd: str):
    url = f"{GCS_BASE_URL}/{cmd}"
    try:
        res = requests.post(url, timeout=1)
        if res.ok:
            logging.info(f"✅ Sent command: {cmd}")
        else:
            logging.warning(f"⚠️ Command failed: {cmd} (status {res.status_code})")
    except Exception as e:
        logging.error(f"❌ Error sending command '{cmd}': {e}")

def main():
    global last_gesture, last_time
    logging.info("🖐️ Gesture control started using model with classes 1-4")
    try:
        for result in model.predict(source=0, stream=True, show=True):
            now = time.time()
            for box in result.boxes:
                cls = int(box.cls[0])
                gesture = model.names[cls]

                if gesture not in gesture_to_command:
                    continue

                if gesture == last_gesture and (now - last_time) < COOLDOWN_SECONDS:
                    continue  # Still cooling down

                send_command(gesture_to_command[gesture])
                last_gesture = gesture
                last_time = now

    except KeyboardInterrupt:
        logging.info("🛑 Gesture control stopped")
    except Exception as e:
        logging.error(f"🔥 Fatal error: {e}")

if __name__ == "__main__":
    main()
