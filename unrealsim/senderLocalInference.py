import sys
import time
import cv2
import csv
import os
import math
from datetime import datetime
import psutil
import threading
import numpy as np
from ultralytics import YOLO
from PIL import Image
import io

# FPS switching list
fps_choices = [20, 22, 24, 26, 28, 30, 32, 34, 36]

# ✅ Default settings   
USE_VIDEO = False
VIDEO_PATH = "unrealsim/videos/nrealv2_640x480.mp4"
ANALYTICS = True
ANALYTICSFILE = 'benchmark'
JPEG_QUALITY = 50
WIDTH = 640
HEIGHT = 480
DISPLAY_FRAME = False
RASPICAM = False
REPLAY_VIDEO = False
FRAMELIMIT = 17000
MODEL_PATH = "unrealsim/models/unrealsim.pt"
POWERCPU = 0
framesPerfps = 100
SCAN_HEIGHTS = [0.2, 0.4, 0.6, 0.8]

# ✅ Command-line parsing (same as before)
for arg in sys.argv[1:]:
    if arg.startswith("USE_VIDEO="):
        USE_VIDEO = arg.split("=")[1].lower() == "true"
    elif arg.startswith("VIDEO_PATH="):
        VIDEO_PATH = arg.split("=", 1)[1]
    elif arg.startswith("MODEL="):
        MODEL_PATH = arg.split("=", 1)[1]
    elif arg.startswith("ANALYTICS="):
        ANALYTICS = arg.split("=")[1].lower() == "true"
    elif arg.startswith("JPEG_QUALITY="):
        JPEG_QUALITY = int(arg.split("=")[1])
    elif arg.startswith("WIDTH="):
        WIDTH = int(arg.split("=")[1])
    elif arg.startswith("HEIGHT="):
        HEIGHT = int(arg.split("=")[1])
    elif arg.startswith("DISPLAY_FRAME="):
        DISPLAY_FRAME = arg.split("=")[1].lower() == "true"
    elif arg.startswith("RASPICAM="):
        RASPICAM = arg.split("=")[1].lower() == "true"
    elif arg.startswith("REPLAY_VIDEO="):
        REPLAY_VIDEO = arg.split("=")[1].lower() == "true"
    elif arg.startswith("POWERCPU="):
        POWERCPU = int(arg.split("=")[1])
    elif arg.startswith("ANALYTICSFILE="):
        ANALYTICSFILE = arg.split("=")[1]

print(f"USE_VIDEO: {USE_VIDEO}")
print(f"VIDEO_PATH: {VIDEO_PATH}")
print(f"ANALYTICS: {ANALYTICS}")
print(f"JPEG_QUALITY: {JPEG_QUALITY}")
print(f"WIDTH: {WIDTH}, HEIGHT: {HEIGHT}")
print(f"DISPLAY_FRAME: {DISPLAY_FRAME}")
print(f"RASPICAM: {RASPICAM}")
print(f"MODEL: {MODEL_PATH}")
print(f"POWERCPU: {POWERCPU}")

# ✅ Load model
model = YOLO(MODEL_PATH, verbose=True)

# ✅ Video source
if RASPICAM:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.start()
else:
    capture = cv2.VideoCapture(VIDEO_PATH if USE_VIDEO else 0)

# ✅ MQTT power reading
LATEST_POWER_MQTT = 0

def mqtt_thread():
    import paho.mqtt.client as mqtt
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connected to MQTT broker")
            client.subscribe("MasterThesis/edgepower")
        else:
            print(f"❌ MQTT connect failed: {rc}")

    def on_message(client, userdata, msg):
        global LATEST_POWER_MQTT
        try:
            LATEST_POWER_MQTT = float(msg.payload.decode())
        except:
            LATEST_POWER_MQTT = 0

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("broker.emqx.io", 1883, 60)
    client.loop_forever()

if POWERCPU == 0:
    threading.Thread(target=mqtt_thread, daemon=True).start()

# ✅ Analytics setup
if ANALYTICS:
    os.makedirs("unrealsim/analytics", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"unrealsim/analytics/{ANALYTICSFILE}_{timestamp_str}.csv"
    with open(csv_filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "datetime",
            "resolution",
            "avg_fps",
            "avg_size_kb",
            "avg_latency_ms",
            "inference_time_ms",
            "processing_time_ms",
            "poweruse_W",
            "queuesize",
            "frame_id"
        ])

# ✅ Loop variables
frame_id = 0
fps_timer = time.time()
fps_count = 0
slot_start_time = time.time()
acc = {"fps": [], "inference": [], "size": [], "power": [], "latency": [], "processing": []}
currentFPS = 0
currentFrameCount = 0
nrFps = len(fps_choices)
prev_frame_time = None
inference_time_ms = 0

while True:
    frame_id += 1
    frame_start = time.time()

    if prev_frame_time is not None:
        latency_ms = (frame_start - prev_frame_time) * 1000
    else:
        latency_ms = 0
    prev_frame_time = frame_start


    processing_ms = latency_ms - inference_time_ms

    currentFrameCount += 1

    if USE_VIDEO:
        ret, frame = capture.read()
        if not ret:
            if REPLAY_VIDEO:
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            else:
                print("End of video.")
                break
    else:
        if RASPICAM:
            frame = picam2.capture_array()
            ret = True
        else:
            ret, frame = capture.read()
    if not ret:
        continue

    frame = cv2.resize(frame, (WIDTH, HEIGHT))

    # Inference
    inf_start = time.time()
    results = model(frame, conf=0.5, verbose=False)
    inf_end = time.time()
    inference_time_ms = (inf_end - inf_start) * 1000

    # Direction detection
    overlay = frame.copy()
    height, width = frame.shape[:2]
    midpoints = []

    for result in results:
        if result.masks is not None:
            mask = result.masks.data[0].cpu().numpy()
            mask = (mask * 255).astype(np.uint8)
            mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

            for r in SCAN_HEIGHTS:
                y = int(height * r)
                scan_row = mask_resized[y, :]
                indices = np.where(scan_row > 0)[0]
                if len(indices) > 0:
                    midpoint_x = int(np.mean(indices))
                    midpoints.append((midpoint_x, y))
                    cv2.circle(overlay, (midpoint_x, y), 5, (255, 0, 0), -1)
                cv2.line(overlay, (0, y), (width, y), (150, 150, 150), 1)

    direction_angle = None
    if midpoints:
        avg_x = int(np.mean([pt[0] for pt in midpoints]))
        target_point = (avg_x, min([pt[1] for pt in midpoints]))
        start_point = (width // 2, height)
        cv2.arrowedLine(overlay, start_point, target_point, (0, 0, 255), 5, tipLength=0.2)
        dx = avg_x - start_point[0]
        dy = start_point[1] - target_point[1]
        angle_rad = np.arctan2(dy, dx)
        direction_angle = np.degrees(angle_rad)

    # FPS
    fps_count += 1
    elapsed = time.time() - fps_timer
    fps = fps_count / elapsed if elapsed > 0 else 0

    cpu_percent = psutil.cpu_percent(interval=None)
    power_cpu_est = round(POWERCPU * cpu_percent * 0.01, 2) if POWERCPU > 0 else 0
    poweruse = power_cpu_est if POWERCPU > 0 else LATEST_POWER_MQTT

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(frame_rgb)
    compressed_image_io = io.BytesIO()
    pil_image.save(compressed_image_io, format="JPEG", quality=JPEG_QUALITY)
    compressed_bytes = compressed_image_io.getvalue()
    size_kb = len(compressed_bytes) / 1024

    acc["fps"].append(fps)
    acc["inference"].append(inference_time_ms)
    acc["size"].append(size_kb)
    acc["power"].append(poweruse)
    acc["latency"].append(latency_ms)
    acc["processing"].append(processing_ms)


    if (time.time() - slot_start_time >= 1.0  or 1==1):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if ANALYTICS:
            with open(csv_filename, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([
                    now,
                    f"{WIDTH}x{HEIGHT}",
                    round(sum(acc["fps"]) / len(acc["fps"]), 2),
                    round(sum(acc["size"]) / len(acc["size"]), 2),
                    round(sum(acc["latency"]) / len(acc["latency"]), 2),
                    round(sum(acc["inference"]) / len(acc["inference"]), 2),
                    round(sum(acc["processing"]) / len(acc["processing"]), 2),
                    round(sum(acc["power"]) / len(acc["power"]), 2),
                    0,
                    frame_id
                ])
        acc = {k: [] for k in acc}
        slot_start_time = time.time()
        fps_timer = time.time()
        fps_count = 0

    if DISPLAY_FRAME == True:
        display = overlay.copy()
        cv2.putText(display, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(display, f"Inference: {inference_time_ms:.1f} ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(display, f"Processing: {processing_ms:.1f} ms", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(display, f"End-to-End-Latency: {latency_ms:.1f} ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        if direction_angle is not None:
            cv2.putText(display, f"Direction: {direction_angle:.1f} deg", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(display, f"Power: {poweruse} W", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.imshow("Local Inference", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    frame_time = time.time() - frame_start

    if USE_VIDEO and frame_id >= FRAMELIMIT:
        break

if not RASPICAM:
    capture.release()
cv2.destroyAllWindows()
print("✅ Program ended.")
