import sys
import asyncio
import json
from sympy import true
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
from playsound import playsound
import simpleaudio as sa
import threading
import numpy as np



# ✅ Standaardinstellingen
USE_VIDEO = False  # True = video, False = webcam
VIDEO_PATH = "unrealsim/videos/nrealv2_640x480.mp4"
MAX_FPS = 20
SIGNALING_SERVER = "ws://192.168.0.74:9000"
ANALYTICS = False  # 🔑 Analytics aan of uit
JPEG_QUALITY = 50
WIDTH = 640
HEIGHT = 480
DISPLAY_FRAME = True  # True = frame tonen, False = zwart scherm
PLAY_SOUND = False  # True = geluid afspelen bij paddetectie
FULLSCREEN = False  
PATH_DETECTED = False  # Global variable to track if a path is detected
RASPICAM = True  # True = Raspicam, False = webcam
lastPathDetected = time.time()  # Timestamp of the last path detection
missedFrames = 0  # Counter for missed frames
successFullFrames = 0
nr_frames = 0  # Counter for successful frames
REPLAY_VIDEO = False  # True = replay video after end, False = stop after last frame



# ✅ Commandline parsing
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
            WIDTH = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige WIDTH waarde, standaard blijft:", WIDTH)
    elif arg.startswith("HEIGHT="):
        try:
            HEIGHT = int(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige HEIGHT waarde, standaard blijft:", HEIGHT)
    elif arg.startswith("PLAY_SOUND="):
        PLAY_SOUND = arg.split("=")[1]
    elif arg.startswith("DISPLAY_FRAME="):
        DISPLAY_FRAME = arg.split("=")[1]
    elif arg.startswith("RASPICAM="):
        RASPICAM = arg.split("=")[1]
    elif arg.startswith("REPLAY_VIDEO="):
        REPLAY_VIDEO = arg.split("=")[1]
    elif arg.startswith("FULLSCREEN="):
        FULLSCREEN = arg.split("=")[1].lower() == "true"

print(f"Signaling Server: {SIGNALING_SERVER}")
print(f"USE_VIDEO: {USE_VIDEO}")
print(f"VIDEO_PATH: {VIDEO_PATH}")
print(f"MAX_FPS: {MAX_FPS}")
print(f"ANALYTICS: {ANALYTICS}")
print(f"JPEG_QUALITY: {JPEG_QUALITY}")
print(f"width: {WIDTH}")
print(f"height: {HEIGHT}")
print(f"FULLSCREEN: {FULLSCREEN}")
print(f"PLAY_SOUND: {PLAY_SOUND}")
print(f"DISPLAY_FRAME: {DISPLAY_FRAME}")

if RASPICAM == True:
    from picamera2 import Picamera2
    # Initialiseer camera
    picam2 = Picamera2()
    picam2.start()


AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

capture = cv2.VideoCapture(VIDEO_PATH if USE_VIDEO else 0)
frame_id = 0
frame_records = {}
latency_ms = 0
DIRECTION_ANGLE = None
should_exit = False

if ANALYTICS:
    os.makedirs("unrealsim/analytics", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"unrealsim/analytics/benchmark_{timestamp_str}.csv"
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            "datetime", "avg_latency_ms", "avg_fps", "avg_size_kb",
            "avg_compression_ms", "avg_encryption_ms",
            "resolution", "max_fps", "signaling_server",
            "jpeg_quality", "width", "height","missed_frames","successful_frames", "last_frame_id"
        ])
    acc = { "latency": [], "fps": [], "size": [], "compression": [], "encryption": [], "missed_frames": [], "successful_frames": [], "last_frame_id": None   }
    slot_start_time = time.time()

def play_sound(sound_file):
    def _play():
        wave_obj = sa.WaveObject.from_wave_file(sound_file)
        play_obj = wave_obj.play()
        play_obj.wait_done()
    threading.Thread(target=_play, daemon=True).start()

def encrypt_data(plain_text):
    encrypt_start_time = time.perf_counter()
    iv = os.urandom(16)
    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(iv), backend=default_backend())
    encryptor = cipher.encryptor()
    padder = padding.PKCS7(algorithms.AES.block_size).padder()
    padded_data = padder.update(plain_text) + padder.finalize()
    encrypted_data = encryptor.update(padded_data) + encryptor.finalize()
    encrypt_end_time = time.perf_counter()
    encryption_time = (encrypt_end_time - encrypt_start_time) * 1_000  # in milliseconds
    return base64.b64encode(iv + encrypted_data).decode('utf-8'), encryption_time

async def send_messages(websocket):
    global frame_id, JPEG_QUALITY, DIRECTION_ANGLE, frame_records, latency_ms, should_exit,missedFrames,successFullFrames,nr_frames,REPLAY_VIDEO
    if ANALYTICS:
        global acc, slot_start_time

    frame_delay = 1.0 / MAX_FPS
    cv2.namedWindow("Video Stream", cv2.WINDOW_NORMAL)
    if FULLSCREEN:
        cv2.setWindowProperty("Video Stream", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    fps_frame_count = 0
    fps_timer_start = time.time()
    fps = 0.0


    while not should_exit:
        frame_id += 1
        frame_start = time.time()

        if (USE_VIDEO == False):
            if RASPICAM == True:
                frame = picam2.capture_array()
                ret= True
            else:
                ret, frame = capture.read()

        
        if USE_VIDEO:
            #capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = capture.read()
            nr_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

            if not ret or frame is None:
                print("Frame niet beschikbaar, probeer volgende frame")
                continue

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        display = frame.copy()
        if DISPLAY_FRAME == False:
            display = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        
        if (PATH_DETECTED == False):
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
        cv2.putText(display, f"path detected: {PATH_DETECTED}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(display, f"missed frames: {missedFrames}", (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(display, f"successful frames: {successFullFrames}", (10, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if (USE_VIDEO == True): 
            cv2.putText(display, f"frame id: {frame_id} / {nr_frames}", (10, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            cv2.putText(display, f"frame id: {frame_id}", (10, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if (frame_id >= nr_frames and USE_VIDEO and REPLAY_VIDEO == False):
            print("✅ Alle frames van de video zijn verzonden, opnieuw beginnen.")
            should_exit = True

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("⏹️ Afsluiten door gebruiker")
            should_exit = True
            break

        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        t0 = time.perf_counter()
        pil_image = Image.fromarray(frame_rgb)
        compressed_image_io = io.BytesIO()
        pil_image.save(compressed_image_io, format="JPEG", quality=JPEG_QUALITY)
        t1 = time.perf_counter()
        compressed_bytes = compressed_image_io.getvalue()
        size_kb = len(compressed_bytes) / 1024
        compression_time = (t1 - t0) * 1_000  # microseconds


        encrypted_data, encryption_time = encrypt_data(compressed_bytes)

        cv2.putText(display, f"Encryption time: {encryption_time:.3f} ms", (10, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(display, f"Compression time: {compression_time:.3f} ms", (10, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Video Stream", display)



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
                        SIGNALING_SERVER,
                        missedFrames,
                        successFullFrames,
                        frame_id
                    ])
                acc = {k: [] for k in acc}
                slot_start_time = time.time()
                missedFrames = 0  # Reset missed frames counter every 10 seconds
                successFullFrames = 0  # Reset successful frames counter every 10 seconds

        message = {
            "frame_id": frame_id,
            "data": encrypted_data,
            "resolution": f"{WIDTH}x{HEIGHT}",
            "size_kb": size_kb,
            "compression_time_ms": compression_time,
            "encryption_time_ms": encryption_time,
            "timestamp": timestamp,
            "missed_frames": missedFrames
        }
        frame_records[frame_id] = {'timestamp': time.time()}
        await websocket.send(json.dumps(message))

        elapsed = time.time() - frame_start
        sleep_time = max(0, frame_delay - elapsed)
        await asyncio.sleep(sleep_time)

async def receive_messages(websocket):
    global JPEG_QUALITY, DIRECTION_ANGLE, frame_records, latency_ms, should_exit,PATH_DETECTED,lastPathDetected,PLAY_SOUND, missedFrames, successFullFrames
    while not should_exit:
        try:
            message = await websocket.recv()
            message_json = json.loads(message)
            if 'quality' in message_json and (1 <= message_json['quality'] <= 100):
                JPEG_QUALITY = message_json['quality']
                print(f"SET JPEG_QUALITY: {JPEG_QUALITY}")
            if 'direction_angle' in message_json:
                DIRECTION_ANGLE = message_json['direction_angle']
                PATH_DETECTED = True
                lastPathDetected = time.time()
                successFullFrames += 1
            if 'detected' in message_json:
                if not message_json['detected']:
                    missedFrames += 1
                    if ((time.time() - lastPathDetected) > 1.0):
                        PATH_DETECTED = False
                        DIRECTION_ANGLE = None
                        if (PLAY_SOUND):
                            play_sound("sounds/beep.wav")
                        lastPathDetected = time.time()
            if 'frame_id' in message_json:
                FRAME_ID = message_json['frame_id']
                received = time.time()
                if FRAME_ID in frame_records:
                    latency_ms = (received - frame_records[FRAME_ID]['timestamp']) * 1000
        except websockets.exceptions.ConnectionClosed:
            print("🚫 Verbinding met server gesloten")
            should_exit = True
            break

async def main():
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        await asyncio.gather(
            send_messages(websocket),
            receive_messages(websocket)
        )
    print("⏹️ Afsluiten door indrukken van 'q' of normale beëindiging.")

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("⏹️ Afsluiten door KeyboardInterrupt...")
    exit(0)
finally:
    capture.release()
    cv2.destroyAllWindows()
    print("✅ Programma netjes afgesloten.")