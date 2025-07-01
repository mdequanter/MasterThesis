import cv2
import numpy as np
import time
import os
from datetime import datetime
import sys

CAMERA_INDEX = 0
FPS = 30
FRAME_SIZE = (640, 480)
OUTPUT_PATH = 'recording/video_output.avi'
SNAPSHOT_DIR = 'train/recording'
RASPICAM = True  # True = Raspicam, False = webcam

if RASPICAM:
    from picamera2 import Picamera2
    # Initialiseer camera
    picam2 = Picamera2()
    picam2.start()


# ✅ Command-line argumenten verwerken
for arg in sys.argv[1:]:
    if arg.startswith("CAMERA="):
        try:
            CAMERA_INDEX = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ CAMERA index ongeldig, standaard blijft 0")
    if arg.startswith("SNAPSHOT_DIR="):
        SNAPSHOT_DIR = int(arg.split("=")[1])

print(f"🎥 Gebruik camera index: {CAMERA_INDEX}")
print (f"📸 Snapshot directory: {SNAPSHOT_DIR}")

# ✅ Output map maken
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

# ✅ Webcam openen

if not RASPICAM:
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])

    if not cap.isOpened():
        print("❌ Kon de camera niet openen.")
        sys.exit(1)

# ✅ VideoWriter setup
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
is_recording = False
prev_time = time.time()
current_frame = None

print("Toetsen: [r]=record [p]=pause [s]=snapshot [q]=quit")

while True:

    if RASPICAM:
        frame = picam2.capture_array()
        ret= True
    else:
        ret, frame = cap.read()


    if not ret:
        print("⚠️ Geen frame ontvangen.")
        break

    frame = cv2.resize(frame, FRAME_SIZE)
    current_frame = frame.copy()

    # FPS berekening
    now = time.time()
    fps_text = 1 / (now - prev_time)
    prev_time = now
    display_frame = frame.copy()
    cv2.putText(display_frame, f"FPS: {fps_text:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Live Camera Feed", display_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("⏹️ Programma afsluiten...")
        break
    elif key == ord('r') and not is_recording:
        out = cv2.VideoWriter(OUTPUT_PATH, fourcc, FPS, FRAME_SIZE)
        is_recording = True
        print(f"⏺️ Start opname: {OUTPUT_PATH}")
    elif key == ord('p') and is_recording:
        is_recording = False
        if out:
            out.release()
        print("⏸️ Opname gepauzeerd.")
    elif key == ord('s') and current_frame is not None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        snapshot_path = os.path.join(SNAPSHOT_DIR, f'snapshot_{timestamp}.jpg')
        cv2.imwrite(snapshot_path, current_frame)
        print(f"📸 Snapshot opgeslagen: {snapshot_path}")

    if is_recording and out:
        out.write(frame)

# Cleanup
cap.release()
if out:
    out.release()
cv2.destroyAllWindows()
print("✅ Programma correct beëindigd.")
