import sys
import asyncio
import json
import websockets
import time
import cv2
import numpy as np
import base64
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend
from collections import deque
from ultralytics import YOLO

# ✅ Settings
screenOutput = True
minSegmentSize = 5000  # Minimale grootte (in pixels) van het segment voor het tekenen van een pijl

frame_times = deque(maxlen=100)
SIGNALING_SERVER = "ws://192.168.0.74:9000"
if len(sys.argv) > 1:
    SIGNALING_SERVER = sys.argv[1]

wantedFramerate = 8
maxQuality = 60
TARGET_WIDTH, TARGET_HEIGHT = 640, 480
AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

model = YOLO('unrealsim/models/blindnavUnreal.pt', verbose=True)

def decrypt_data(encrypted_base64):
    encrypted_data = base64.b64decode(encrypted_base64)
    iv = encrypted_data[:16]
    encrypted_bytes = encrypted_data[16:]

    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(iv), backend=default_backend())
    decryptor = cipher.decryptor()

    decrypted_padded = decryptor.update(encrypted_bytes) + decryptor.finalize()
    unpadder = padding.PKCS7(algorithms.AES.block_size).unpadder()
    decrypted_bytes = unpadder.update(decrypted_padded) + unpadder.finalize()
    return decrypted_bytes

async def receive_messages():
    quality = 50
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met Signaling Server: {SIGNALING_SERVER}")
        frameCounter = 0
        last_executed_q = time.time()
        last_status_time = time.time()

        while True:
            try:
                message = await websocket.recv()
                message_json = json.loads(message)
                frameCounter += 1

                decrypted_data = decrypt_data(message_json["data"])
                np_arr = np.frombuffer(decrypted_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                frame_id = message_json["frame_id"]

                if frame is None:
                    continue

                start_inference = time.time()
                results = model(frame, conf=0.1, verbose=False)
                end_inference = time.time()
                inference_time = (end_inference - start_inference) * 1000

                overlay = frame.copy()
                largest_area = 0
                largest_center = None
                direction_angle = None

                for result in results:
                    if result.masks is not None:
                        for mask in result.masks.xy:
                            points = np.array(mask, dtype=np.int32)
                            area = cv2.contourArea(points)
                            if area > largest_area:
                                M = cv2.moments(points)
                                if M["m00"] != 0:
                                    cX = int(M["m10"] / M["m00"])
                                    cY = int(M["m01"] / M["m00"])
                                    largest_center = (cX, cY)
                                    largest_area = area
                            cv2.fillPoly(overlay, [points], color=(0, 255, 0))
                            cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)

                alpha = 0.3
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

                if largest_area >= minSegmentSize and largest_center is not None:
                    start_point = (frame.shape[1] // 2, frame.shape[0])
                    end_point = largest_center
                    cv2.arrowedLine(frame, start_point, end_point, (0, 0, 255), 5, tipLength=0.2)

                    # Bereken richting in graden
                    dx = largest_center[0] - start_point[0]
                    dy = start_point[1] - largest_center[1]
                    angle_rad = np.arctan2(dy, dx)
                    angle_deg = np.degrees(angle_rad)

                    # Beperk naar -90 tot 90 graden
                    direction_angle = angle_deg
                    # Stuur richting naar de server
                    await websocket.send(json.dumps({"direction_angle": round(direction_angle, 2), "frame_id": frame_id}))

                current_time = time.time()
                frame_times.append(current_time)
                while frame_times and current_time - frame_times[0] > 1.0:
                    frame_times.popleft()
                fps_display = len(frame_times)

                lastQuality = quality
                if fps_display > wantedFramerate and frameCounter > 200:
                    if current_time - last_executed_q >= 0.2:
                        quality = min(quality + 10, maxQuality)
                        if quality != lastQuality:
                            print(f"📈 verhoog kwaliteit naar {quality}")
                            await websocket.send(json.dumps({"quality": quality}))
                        last_executed_q = current_time
                if fps_display < wantedFramerate - 2 and frameCounter > 200:
                    if current_time - last_executed_q >= 0.2:
                        quality = max(quality - 10, 1)
                        if quality != lastQuality:
                            print(f"📉 verlaag kwaliteit naar {quality}")
                            await websocket.send(json.dumps({"quality": quality}))
                        last_executed_q = current_time

                if current_time - last_status_time >= 5.0:
                    print(f"🟢 Verbinding actief - ontvangen frames: {frameCounter}")
                    last_status_time = current_time

                if screenOutput:
                    cv2.putText(frame, f"Time: {message_json['timestamp']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Resolution: {message_json['resolution']}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Size: {round(message_json['size_kb'], 2)} KB", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Comp. Time({quality})%: {round(message_json['compression_time_ms'], 2)} ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Encryption: {round(message_json['encryption_time_ms'], 2)} ms", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"FPS: {fps_display}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                    cv2.putText(frame, f"Inf.Time: {inference_time:.2f} ms", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                    if direction_angle is not None:
                        cv2.putText(frame, f"Direction: {round(direction_angle,2)} waarde", (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    cv2.imshow("Ontvangen + Segmentatie + Richting", frame)
                    cv2.waitKey(1)
                    
            except websockets.exceptions.ConnectionClosed:
                print("🚫 Verbinding met server gesloten")
                break

asyncio.run(receive_messages())
