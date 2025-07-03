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
MODEL = 'unrealsim/models/unrealsim.pt'
SIGNALING_SERVER = "ws://192.168.0.74:9000"
DETECTION_CONFIDENCE = 0.85
frame_times = deque(maxlen=100)
SCAN_HEIGHTS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

# ✅ Commandline parsing
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]
    elif arg.startswith("MODEL="):
        try:
            MODEL = arg.split("=")[1]
        except ValueError:
            print("⚠️ Ongeldige MODEL waarde, standaard blijft:", MODEL)

print(f"Signaling Server: {SIGNALING_SERVER}")
print(f"MODEL: {MODEL}")

wantedFramerate = 8
maxQuality = 60
TARGET_WIDTH, TARGET_HEIGHT = 640, 480
AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

model = YOLO(MODEL, verbose=True)

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
    global DETECTION_CONFIDENCE
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
                results = model(frame, conf=DETECTION_CONFIDENCE, verbose=False)
                end_inference = time.time()
                inference_time = (end_inference - start_inference) * 1000

                overlay = frame.copy()
                height, width = frame.shape[:2]
                midpoints = []

                for result in results:
                    if result.masks is not None:
                        mask = result.masks.data[0].cpu().numpy()
                        mask = (mask * 255).astype(np.uint8)

                        mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
                        green_overlay = np.full_like(frame, (0, 255, 0))
                        blended = cv2.addWeighted(frame, 0.3, green_overlay, 0.7, 0)
                        overlay[mask_resized > 0] = blended[mask_resized > 0]

                        for r in SCAN_HEIGHTS:
                            y = int(height * r)
                            if y >= height:
                                continue
                            scan_row = mask_resized[y, :]
                            indices = np.where(scan_row > 0)[0]
                            if len(indices) > 0:
                                midpoint_x = int(np.mean(indices))
                                midpoints.append((midpoint_x, y))
                                cv2.circle(overlay, (midpoint_x, y), 5, (255, 0, 0), -1)
                            cv2.line(overlay, (0, y), (width, y), (150, 150, 150), 1)

                direction_angle = 90
                if midpoints:
                    avg_x = int(np.mean([pt[0] for pt in midpoints]))
                    target_point = (avg_x, min([pt[1] for pt in midpoints]))
                    start_point = (width // 2, height)
                    cv2.arrowedLine(overlay, start_point, target_point, (0, 0, 255), 5, tipLength=0.2)
                    dx = avg_x - start_point[0]
                    dy = start_point[1] - target_point[1]
                    angle_rad = np.arctan2(dy, dx)
                    direction_angle = np.degrees(angle_rad)
                    detected = True
                    await websocket.send(json.dumps({"direction_angle": round(direction_angle, 2), "frame_id": frame_id}))
                else:
                    await websocket.send(json.dumps({"detected": False, "frame_id": frame_id}))

                

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
                    cv2.putText(overlay, f"Time: {message_json['timestamp']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Resolution: {message_json['resolution']}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Size: {round(message_json['size_kb'], 2)} KB", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Comp. Time({quality})%: {round(message_json['compression_time_ms'], 2)} ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"Encryption: {round(message_json['encryption_time_ms'], 2)} ms", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(overlay, f"FPS: {fps_display}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                    cv2.putText(overlay, f"Inf.Time: {inference_time:.2f} ms", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                    cv2.putText(overlay, f"Direction: {round(direction_angle,2)} waarde", (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.imshow("Ontvangen + Segmentatie + Richting", overlay)
                    cv2.waitKey(1)

            except websockets.exceptions.ConnectionClosed:
                print("🚫 Verbinding met server gesloten")
                break

asyncio.run(receive_messages())
