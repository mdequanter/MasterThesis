import sys
import asyncio
import json
import websockets
import time
import cv2
import base64
import io
from PIL import Image
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend
import os
import math

# ✅ Instellingen
USE_VIDEO = True  # True = video, False = webcam
VIDEO_PATH = "unrealsim/videos/UnrealParkRecording.mp4"
MAX_FPS = 20 # Max aantal frames per seconde

SIGNALING_SERVER = "ws://192.168.0.74:9000"
#SIGNALING_SERVER = "ws://heliwi.duckdns.org:9000"

if len(sys.argv) > 1:
    SIGNALING_SERVER = sys.argv[1]

print(f"Signaling Server: {SIGNALING_SERVER}")

JPEG_QUALITY = 50
width = 800
height = 440


AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

# Capture openen
capture = cv2.VideoCapture(VIDEO_PATH if USE_VIDEO else 0)
frame_id = 0
frame_records = {}
latency_ms = 0  # Variabele om latency bij te houden




DIRECTION_ANGLE = None  # Globale variabele om richting bij te houden

def encrypt_data(plain_text):
    encrypt_start_time = time.time()
    iv = os.urandom(16)
    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(iv), backend=default_backend())
    encryptor = cipher.encryptor()
    padder = padding.PKCS7(algorithms.AES.block_size).padder()
    padded_data = padder.update(plain_text) + padder.finalize()
    encrypted_data = encryptor.update(padded_data) + encryptor.finalize()
    encrypt_end_time = time.time()
    return base64.b64encode(iv + encrypted_data).decode('utf-8'), (encrypt_end_time - encrypt_start_time) * 1000

async def send_messages(websocket):
    global frame_id, JPEG_QUALITY, DIRECTION_ANGLE, frame_records
    frame_delay = 1.0 / MAX_FPS
    global DIRECTION_ANGLE
    
    #cv2.namedWindow("Video Stream", cv2.WND_PROP_FULLSCREEN)
    #cv2.setWindowProperty("Video Stream", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    
    while True:
        frame_id+=1
        frame_start = time.time()
        ret, frame = capture.read()
        if not ret:
            print("❌ Geen frame opgehaald")
            if USE_VIDEO:
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            else:
                break

        frame = cv2.resize(frame, (width, height))
        display = frame.copy()
        # ✅ Teken pijl als DIRECTION_ANGLE beschikbaar is
        if DIRECTION_ANGLE is not None:
            center_x = width // 2
            center_y = height
            length = 100  # Lengte van de pijl

            rad = math.radians(DIRECTION_ANGLE)
            end_x = int(center_x + length * math.cos(rad))
            end_y = int(center_y - length * math.sin(rad))
            cv2.arrowedLine(display, (center_x, center_y), (end_x, end_y), (0, 0, 255), 5, tipLength=0.2)
            cv2.putText(display, f"direction: {DIRECTION_ANGLE} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(display, f"latency: {latency_ms:.2f} ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)


        # ✅ Toon de stream op het scherm
        cv2.imshow("Video Stream", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("⏹️ Afsluiten door gebruiker")
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)

        compressed_image_io = io.BytesIO()
        t0 = time.time()
        pil_image.save(compressed_image_io, format="JPEG", quality=JPEG_QUALITY)
        t1 = time.time()
        compressed_bytes = compressed_image_io.getvalue()
        size_kb = len(compressed_bytes) / 1024
        compression_time = (t1 - t0) * 1000

        encrypted_data, encryption_time = encrypt_data(compressed_bytes)

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        message = {
            "frame_id": frame_id,
            "data": encrypted_data,
            "timestamp": timestamp,
            "resolution": f"{width}x{height}",
            "size_kb": round(size_kb, 2),
            "compression_time_ms": round(compression_time, 2),
            "encryption_time_ms": round(encryption_time, 2)
        }

        frame_records[frame_id] = {'timestamp': time.time()}

        await websocket.send(json.dumps(message))



        # FPS limiter
        elapsed = time.time() - frame_start
        sleep_time = max(0, frame_delay - elapsed)
        await asyncio.sleep(sleep_time)

async def receive_messages(websocket):
    global JPEG_QUALITY, DIRECTION_ANGLE, frame_records,latency_ms
    while True:
        try:
            message = await websocket.recv()
            message_json = json.loads(message)
            if 'quality' in message_json and (1 <= message_json['quality'] <= 100):
                JPEG_QUALITY = message_json['quality']
                print(f"SET JPEG_QUALITY: {JPEG_QUALITY}")
            if 'direction_angle' in message_json:
                DIRECTION_ANGLE = message_json['direction_angle']
                #print(f"DIRECTION ANGLE: {DIRECTION_ANGLE}")
            if 'frame_id' in message_json:
                FRAME_ID = message_json['frame_id']
                received = time.time()
                frame_records[FRAME_ID]['received'] = received
                latency_ms = (received - frame_records[FRAME_ID]['timestamp']) * 1000
                #print(f"⏱️ Latency frame {FRAME_ID}: {latency_ms:.2f} ms")

        except websockets.exceptions.ConnectionClosed:
            print("🚫 Verbinding met server gesloten")
            break

async def main():
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        await asyncio.gather(
            send_messages(websocket),
            receive_messages(websocket)
        )

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("⏹️ Afsluiten...")
finally:
    capture.release()
    cv2.destroyAllWindows()
