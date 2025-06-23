import sys
import asyncio
import json
import websockets
import time
import cv2
import base64
import io
import csv
from PIL import Image
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend
import os
import math
from datetime import datetime

# ✅ Standaardinstellingen
USE_VIDEO = False  # True = video, False = webcam
VIDEO_PATH = "unrealsim/videos/UnrealParkRecording.mp4"
MAX_FPS = 20
SIGNALING_SERVER = "ws://192.168.0.74:9000"
ANALYTICS = True  # 🔑 Analytics aan of uit
JPEG_QUALITY = 50
WIDTH = 800
HEIGHT = 400

# ✅ Commandline parsing
# Voorbeeld aanroepen:
# python script.py USE_VIDEO=True VIDEO_PATH=unrealsim/videos/Andere.mp4 MAX_FPS=30 SIGNALING_SERVER=ws://127.0.0.1:9000 ANALYTICS=False JPEG_QUALITY=60 WIDTH=640 HEIGHT=480

for arg in sys.argv[1:]:
    if arg.startswith("USE_VIDEO="):
        USE_VIDEO = arg.split("=")[1].lower() == "true"
    elif arg.startswith("VIDEO_PATH="):
        VIDEO_PATH = arg.split("=", 1)[1]
    elif arg.startswith("MAX_FPS="):
        try:
            MAX_FPS = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige MAX_FPS waarde, standaard blijft:", MAX_FPS)
    elif arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]
    elif arg.startswith("ANALYTICS="):
        ANALYTICS = arg.split("=")[1].lower() == "true"
    elif arg.startswith("JPEG_QUALITY="):
        try:
            JPEG_QUALITY = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige JPEG_QUALITY waarde, standaard blijft:", JPEG_QUALITY)
    elif arg.startswith("WIDTH="):
        try:
            width = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige width waarde, standaard blijft:", WIDTH)
    elif arg.startswith("HEIGHT="):
        try:
            height = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige height waarde, standaard blijft:", HEIGHT)

# ✅ Debug print
print(f"Signaling Server: {SIGNALING_SERVER}")
print(f"USE_VIDEO: {USE_VIDEO}")
print(f"VIDEO_PATH: {VIDEO_PATH}")
print(f"MAX_FPS: {MAX_FPS}")
print(f"ANALYTICS: {ANALYTICS}")
print(f"JPEG_QUALITY: {JPEG_QUALITY}")
print(f"width: {WIDTH}")
print(f"height: {HEIGHT}")

AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

capture = cv2.VideoCapture(VIDEO_PATH if USE_VIDEO else 0)
frame_id = 0
frame_records = {}
latency_ms = 0
DIRECTION_ANGLE = None

# Analytics setup
if ANALYTICS:
    os.makedirs("unrealsim/analytics", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"unrealsim/analytics/benchmark_{timestamp_str}.csv"
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            "datetime", "avg_latency_ms", "avg_fps", "avg_size_kb",
            "avg_compression_ms", "avg_encryption_ms",
            "resolution", "max_fps", "signaling_server"
        ])
    acc = { "latency": [], "fps": [], "size": [], "compression": [], "encryption": [] }
    slot_start_time = time.time()

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
    global frame_id, JPEG_QUALITY, DIRECTION_ANGLE, frame_records, latency_ms
    if ANALYTICS:
        global acc, slot_start_time

    frame_delay = 1.0 / MAX_FPS
    cv2.namedWindow("Video Stream", cv2.WINDOW_NORMAL)

    fps_frame_count = 0
    fps_timer_start = time.time()
    fps = 0.0

    while True:
        frame_id += 1
        frame_start = time.time()
        ret, frame = capture.read()
        if not ret:
            print("❌ Geen frame opgehaald")
            if USE_VIDEO:
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            else:
                break

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        display = frame.copy()

        fps_frame_count += 1
        if time.time() - fps_timer_start >= 1.0:
            fps = fps_frame_count / (time.time() - fps_timer_start)
            fps_timer_start = time.time()
            fps_frame_count = 0

        if DIRECTION_ANGLE is not None:
            center_x = WIDTH // 2
            center_y = HEIGHT
            length = 100
            rad = math.radians(DIRECTION_ANGLE)
            end_x = int(center_x + length * math.cos(rad))
            end_y = int(center_y - length * math.sin(rad))
            cv2.arrowedLine(display, (center_x, center_y), (end_x, end_y), (0, 0, 255), 5, tipLength=0.2)
            cv2.putText(display, f"direction: {DIRECTION_ANGLE} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.putText(display, f"latency: {latency_ms:.2f} ms", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(display, f"FPS: {fps:.2f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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

        cv2.imshow("Video Stream", display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("⏹️ Afsluiten door gebruiker")
            break

        if ANALYTICS:
            acc["latency"].append(latency_ms)
            acc["fps"].append(fps)
            acc["size"].append(size_kb)
            acc["compression"].append(compression_time)
            acc["encryption"].append(encryption_time)

            if time.time() - slot_start_time >= 10.0:
                with open(csv_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    writer.writerow([
                        now,
                        round(sum(acc["latency"]) / len(acc["latency"]), 2) if acc["latency"] else 0,
                        round(sum(acc["fps"]) / len(acc["fps"]), 2) if acc["fps"] else 0,
                        round(sum(acc["size"]) / len(acc["size"]), 2) if acc["size"] else 0,
                        round(sum(acc["compression"]) / len(acc["compression"]), 2) if acc["compression"] else 0,
                        round(sum(acc["encryption"]) / len(acc["encryption"]), 2) if acc["encryption"] else 0,
                        f"{WIDTH}x{HEIGHT}",
                        MAX_FPS,
                        SIGNALING_SERVER
                    ])
                acc = {k: [] for k in acc}
                slot_start_time = time.time()

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        message = {
            "frame_id": frame_id,
            "data": encrypted_data,
            "timestamp": timestamp
        }
        frame_records[frame_id] = {'timestamp': time.time()}
        await websocket.send(json.dumps(message))

        elapsed = time.time() - frame_start
        sleep_time = max(0, frame_delay - elapsed)
        await asyncio.sleep(sleep_time)

async def receive_messages(websocket):
    global JPEG_QUALITY, DIRECTION_ANGLE, frame_records, latency_ms
    while True:
        try:
            message = await websocket.recv()
            message_json = json.loads(message)
            if 'quality' in message_json and (1 <= message_json['quality'] <= 100):
                JPEG_QUALITY = message_json['quality']
                print(f"SET JPEG_QUALITY: {JPEG_QUALITY}")
            if 'direction_angle' in message_json:
                DIRECTION_ANGLE = message_json['direction_angle']
            if 'frame_id' in message_json:
                FRAME_ID = message_json['frame_id']
                received = time.time()
                if FRAME_ID in frame_records:
                    latency_ms = (received - frame_records[FRAME_ID]['timestamp']) * 1000
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
