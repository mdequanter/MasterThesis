import sys
import asyncio
import json
import websockets
import time
import cv2
import numpy as np
import base64
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
    if arg.startswith("OUTPUT="):
        screenOutput = arg.split("=", 1)[1] in ("1", "true", "True", "yes")

print(f"Signaling Server: {SIGNALING_SERVER}")
print(f"MODEL: {MODEL}")

wantedFramerate = 8
maxQuality = 60
TARGET_WIDTH, TARGET_HEIGHT = 640, 480

model = YOLO(MODEL, verbose=True)

def decode_message_to_frame(msg):
    """
    msg kan bytes (raw JPEG) of str (JSON met base64 JPEG) zijn.
    Retourneert een OpenCV BGR frame of None.
    """
    try:
        if isinstance(msg, (bytes, bytearray)):
            jpeg_bytes = bytes(msg)
        elif isinstance(msg, str):
            # Verwacht JSON met {"data": "<base64_jpeg>"}
            try:
                payload = json.loads(msg)
                b64 = payload.get("data")
                if not b64:
                    return None
                jpeg_bytes = base64.b64decode(b64)
            except json.JSONDecodeError:
                return None
        else:
            return None

        np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return frame
    except Exception:
        return None

async def receive_messages():
    global DETECTION_CONFIDENCE
    async with websockets.connect(SIGNALING_SERVER, max_size=None) as websocket:
        print(f"✅ Verbonden met Signaling Server: {SIGNALING_SERVER}")

        while True:
            try:
                message = await websocket.recv()
            except websockets.exceptions.ConnectionClosed:
                print("🚫 Verbinding met server gesloten")
                break

            frame = decode_message_to_frame(message)
            if frame is None:
                continue

            start_inference = time.time()
            results = model(frame, conf=DETECTION_CONFIDENCE, verbose=False)
            end_inference = time.time()
            inference_time_ms = (end_inference - start_inference) * 1000.0

            # Optioneel: teken overlay voor debugging
            overlay = frame if screenOutput else None
            height, width = frame.shape[:2]
            midpoints = []

            for result in results:
                if result.masks is not None:
                    # Neem eerste mask (pas aan indien meerdere klassen/instaties)
                    mask = result.masks.data[0].cpu().numpy()
                    mask = (mask * 255).astype(np.uint8)

                    mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

                    if screenOutput:
                        # Groen overlay
                        green_overlay = np.full_like(frame, (0, 255, 0))
                        blended = cv2.addWeighted(frame, 0.3, green_overlay, 0.7, 0)
                        overlay = frame.copy()
                        overlay[mask_resized > 0] = blended[mask_resized > 0]

                    # Scan horizontale stroken en neem midden van positieve pixels
                    for r in SCAN_HEIGHTS:
                        y = int(height * r)
                        if y >= height:
                            continue
                        scan_row = mask_resized[y, :]
                        indices = np.where(scan_row > 0)[0]
                        if len(indices) > 0:
                            midpoint_x = int(np.mean(indices))
                            midpoints.append((midpoint_x, y))
                            if screenOutput:
                                cv2.circle(overlay, (midpoint_x, y), 5, (255, 0, 0), -1)
                        if screenOutput:
                            cv2.line(overlay, (0, y), (width, y), (150, 150, 150), 1)

            # Bepaal heading (graden) — default 90 (rechtdoor)
            direction_angle = 90.0
            if midpoints:
                avg_x = int(np.mean([pt[0] for pt in midpoints]))
                target_point = (avg_x, min([pt[1] for pt in midpoints]))
                start_point = (width // 2, height)
                dx = avg_x - start_point[0]
                dy = start_point[1] - target_point[1]
                angle_rad = np.arctan2(dy, dx)
                direction_angle = float(np.degrees(angle_rad))

                if screenOutput:
                    cv2.arrowedLine(overlay, start_point, target_point, (0, 0, 255), 5, tipLength=0.2)

            # ✉️ Enkel en alleen de heading terugsturen
            try:
                await websocket.send(json.dumps({"heading": round(direction_angle, 2)}))
            except Exception as e:
                print(f"WS send error: {e}")

            if screenOutput:
                # Debug window (druk 'q' om te stoppen)
                cv2.imshow("Segmentation (unencrypted)", overlay if overlay is not None else frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    if screenOutput:
        cv2.destroyAllWindows()

asyncio.run(receive_messages())
