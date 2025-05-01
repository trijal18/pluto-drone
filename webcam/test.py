#!/usr/bin/env python
# hand_gesture_webcam.py

import cv2
import mediapipe as mp

# Initialize mediapipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

# Open webcam (0 = default cam)
cap = cv2.VideoCapture(0)

def detect_gesture(landmarks):
    thumb_tip = landmarks[4]
    index_tip = landmarks[8]
    middle_tip = landmarks[12]
    ring_tip = landmarks[16]
    pinky_tip = landmarks[20]

    if all(lm.y < landmarks[0].y for lm in [index_tip, middle_tip, ring_tip, pinky_tip]):
        return "Open Palm"
    elif all(lm.y > landmarks[0].y for lm in [index_tip, middle_tip, ring_tip, pinky_tip]):
        return "Fist"
    else:
        return "Unknown"

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("❌ Failed to grab frame")
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            lm_list = hand_landmarks.landmark
            gesture = detect_gesture(lm_list)

            cv2.putText(frame, "Gesture: " + gesture, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Hand Gesture Detection", frame)

    if cv2.waitKey(5) & 0xFF == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
