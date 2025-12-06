import sys
import asyncio
import json
import websockets
import time
import cv2
import numpy as np
import base64
import os  # 👈 toegevoegd
from collections import deque
from ultralytics import YOLO

import csv                       
from datetime import datetime    



# ✅ Settings 
CSV_PATH = "androidApp.csv"

screenOutput = False
MODEL = 'unrealsim/models/unrealsim.pt'
SIGNALING_SERVER = "ws://192.168.0.74:9000"
DETECTION_CONFIDENCE = 0.3
frame_times = deque(maxlen=100)
SCAN_HEIGHTS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
standardModel = "unrealsim.pt"
selectedModel = "unrealsim.pt"
selectedModelLast = "unrealsim.pt"

# ✅ Opslag-instellingen (globaal)

recordMode = False            # 👈 of we in recordmode zitten

SAVE_FRAME_EVERY_SECONDS = 30  # 👈 maximaal 1 frame per 30s
FRAMES_DIR = "saved_frames"    # 👈 doelmap voor opgeslagen frames
_last_saved_ts = 0.0           # 👈 interne timestamp (niet aanpassen)

# Zorg dat de map bestaat
os.makedirs(FRAMES_DIR, exist_ok=True)

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


def append_telemetry_row(timestamp_iso, latency_ms, longitude, latitude, model_name , connectionType):
    """
    Schrijft één rij naar telemetry_log.csv.
    Maakt het bestand + header automatisch aan als het nog niet bestaat.
    """
    file_exists = os.path.exists(CSV_PATH)

    # a = append, newline="" om extra lege regels op Windows te vermijden
    with open(CSV_PATH, mode="a", newline="") as f:
        writer = csv.writer(f)
        # Header schrijven als bestand nog niet bestond
        if not file_exists:
            writer.writerow(["timestamp_iso", "latency_ms", "longitude", "latitude", "model", "connectionType"])

        writer.writerow([
            timestamp_iso,
            latency_ms if latency_ms is not None else "",
            longitude if longitude is not None else "",
            latitude if latitude is not None else "",
            model_name if model_name is not None else "",
            connectionType if connectionType is not None else ""
        ])


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
    global DETECTION_CONFIDENCE, model
    global selectedModelLast, selectedModel
    global _last_saved_ts  # 👈 nodig om de throttle-timestamp te wijzigen
    global recordMode    # 👈 om recordMode te wijzigen
    global MODEL

    async with websockets.connect(SIGNALING_SERVER, max_size=None) as websocket:
        print(f"✅ Verbonden met Signaling Server: {SIGNALING_SERVER}")
        global recordMode


        pending_frame_id = None  # wordt gezet door voorafgaande frame_meta

        while True:
            try:
                message = await websocket.recv()
            except websockets.exceptions.ConnectionClosed:
                print("🚫 Verbinding met server gesloten")
                break

            frame_id = None
            frame = None

            # 1) Tekst? → probeer JSON (frame_meta of base64 frame)
            if isinstance(message, str):
                try:
                    payload = json.loads(message)
                    msg_type = payload.get("type")
                    #print(f"inhoud bericht:{payload}")
                    
                    modelConfidence = payload.get("ModelConfidence")
                    if modelConfidence is not None:
                        modelConfidence = float(int(modelConfidence)/100)
                        DETECTION_CONFIDENCE = modelConfidence
                        print(f"Nieuwe detectie-drempel: {DETECTION_CONFIDENCE}")      



                    selectedModel = payload.get("selectedModel")
                    if selectedModel is not None and selectedModel != selectedModelLast:

                        if selectedModel == "recordmode":
                            #selectedModel = standardModel
                            #MODEL = "unrealsim/models/" + standardModel
                            recordMode = True
                            print("Recordmode aan")
                            selectedModelLast = selectedModel
                        else:
                            if (selectedModelLast != selectedModel):
                                print("Recordmode uit")
                                recordMode = False
                                print(f"New model selected: {selectedModel}")
                                selectedModelLast = selectedModel
                                MODEL = "unrealsim/models/" + selectedModelLast

                        model = YOLO(MODEL, verbose=True)

                    if msg_type == "stats":
                        print ("📊 Stats:", payload)


                    if msg_type == "frame_meta":
                        # meta komt vóór de JPEG
                        pending_frame_id = payload.get("frame_id")


                        if (recordMode == True):
                            timestamp_iso = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            latency_ms = payload.get('latency_ms')
                            longitude = payload.get('longitude')
                            latitude = payload.get('latitude')
                            model_name = MODEL.split("/")[-1]
                            connectionType = payload.get('connectionType')
                            
                            append_telemetry_row(timestamp_iso, latency_ms, longitude, latitude, model_name, connectionType )

                            print (f"latency_ms: {latency_ms}, longitude: {longitude}, latitude: {latitude}, selectedModel: {model_name}, connectionType: {connectionType}")
                        # wacht op volgende recv() voor het eigenlijke frame
                        continue

                    # fallback: er komt eventueel ook een JSON met base64 frame
                    if "data" in payload:
                        frame = decode_message_to_frame(message)
                        # pak frame_id als die in payload zit (optioneel)
                        frame_id = payload.get("frame_id", pending_frame_id)
                        pending_frame_id = None
                    else:
                        # Onbekend JSON-bericht → negeren
                        continue
                except json.JSONDecodeError:
                    # Onverwachte tekst → negeren
                    continue

            # 2) Binaire JPEG
            elif isinstance(message, (bytes, bytearray)):
                frame = decode_message_to_frame(message)
                frame_id = pending_frame_id
                pending_frame_id = None

            # Geen bruikbaar frame
            if frame is None:
                continue

            # === Inference ===
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

            # ✉️ Heading + frame_id terugsturen
            try:
                await websocket.send(json.dumps({
                    "heading": round(direction_angle, 2),
                    "frame_id": frame_id
                }))
            except Exception as e:
                print(f"WS send error: {e}")

            # 💾 Throttled frame save (max 1 per SAVE_FRAME_EVERY_SECONDS)
            now = time.time()
            if now - _last_saved_ts >= SAVE_FRAME_EVERY_SECONDS and recordMode==True:
                img_to_save = overlay if (overlay is not None) else frame
                ts_str = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
                fid = f"_{frame_id}" if frame_id is not None else ""
                out_path = os.path.join(FRAMES_DIR, f"frame{fid}_{ts_str}.jpg")
                try:
                    ok = cv2.imwrite(out_path, img_to_save)
                    if ok:
                        _last_saved_ts = now
                    else:
                        print(f"⚠️ Kon frame niet opslaan naar {out_path}")
                except Exception as e:
                    print(f"⚠️ Fout bij opslaan frame: {e}")

            cv2.imshow("Segmentation (unencrypted)", overlay if overlay is not None else frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    if screenOutput:
        cv2.destroyAllWindows()

asyncio.run(receive_messages())
