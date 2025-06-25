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

# ✅ Settings
screenOutput = True  # Zet op False om zonder cv2.imshow te draaien

frame_times = deque(maxlen=100)  # Schuivend venster van tijdstempels

MODEL = 'unrealsim/models/blindnavUnreal.pt'
SIGNALING_SERVER = "ws://192.168.0.74:9000"  # Signaling server URL

frame_times = deque(maxlen=100)

# ✅ Commandline parsing
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]
    elif arg.startswith("MODEL="):
        try:
            MODEL = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige MODEL waarde, standaard blijft:", MODEL)

print(f"Signaling Server: {SIGNALING_SERVER}")
print(f"MODEL: {MODEL}")

wantedFramerate = 25
maxQuality = 60

TARGET_WIDTH, TARGET_HEIGHT = 640, 480
AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

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

                # FPS tracking
                current_time = time.time()
                frame_times.append(current_time)
                while frame_times and current_time - frame_times[0] > 1.0:
                    frame_times.popleft()
                fps_display = len(frame_times)

                lastQuality = quality

                # Kwaliteitsaanpassing
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

                # Status elke 5 seconden
                if current_time - last_status_time >= 5.0:
                    print(f"🟢 Verbinding actief - ontvangen frames: {frameCounter}")
                    last_status_time = current_time

                if screenOutput and frame is not None:
                    cv2.putText(frame, f"Time: {message_json['timestamp']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Resolution: {message_json['resolution']}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Size: {round(message_json['size_kb'], 2)} KB", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Comp. Time({quality})%: {round(message_json['compression_time_ms'], 2)} ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Encryption: {round(message_json['encryption_time_ms'], 2)} ms", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"FPS: {fps_display}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                    cv2.putText(frame, f"Framecounter: {frameCounter}", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                    cv2.imshow("Ontvangen Afbeelding", frame)
                    cv2.waitKey(1)

            except websockets.exceptions.ConnectionClosed:
                print("🚫 Verbinding met server gesloten")
                break

asyncio.run(receive_messages())
