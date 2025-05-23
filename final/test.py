from ultralytics import YOLO
from plutocontrol import pluto  
import time

model = YOLO(r"final\best_5.pt")

# Variables to avoid spamming same command
last_gesture = None
last_action_time = 0
cooldown_seconds = 3  # Adjust as needed

try:
    # Create an instance of the Pluto class
    Pluto = pluto()
    # Connect to the drone
    Pluto.connect()
    time.sleep(5)
except Exception as e:
    print(f"Error occurred: {e}")

try:
    with open('log.txt', 'a') as log_file:
        log_file.write("Starting object detection process...\n")

        while True:
            results = model.predict(source=0, name='test1', exist_ok=True, save_crop=True, show=True, stream=True)

            for result in results:
                current_time = time.time()
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    class_name = model.names[class_id]

                    if class_name != last_gesture and current_time - last_action_time > cooldown_seconds:
                        try:
                            if class_name == 'V':
                                Pluto.arm()
                                print("Drone Armed")
                                log_file.write("Action: Arm\n")
                                last_gesture = 'V'
                                last_action_time = current_time
                            elif class_name == 'W':
                                Pluto.disarm()
                                print("Drone Disarmed")
                                log_file.write("Action: Disarm\n")
                                last_gesture = 'W'
                                last_action_time = current_time
                            else:
                                print(f"Unmapped gesture: {class_name}")
                                log_file.write(f"No mapping for gesture: {class_name}\n")
                                last_gesture = class_name
                                last_action_time = current_time
                        except Exception as move_error:
                            print(f"[ERROR] Movement logic failed: {move_error}")
                            log_file.write(f"[ERROR] Movement logic failed: {move_error}\n")

except Exception as e:
    print(f"Error occurred: {e}")
