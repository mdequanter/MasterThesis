#!/usr/bin/env python3
import cv2
import requests
import time
import io
from typing import Dict, Any

# === CONFIG ===
API_KEY = "19176151-a2ae-4f4e-8dfe-28b15d8bd3c4"  # CompreFace Recognition service key
BASE_URL = "http://192.168.0.79:8000"
ENDPOINT = f"{BASE_URL}/api/v1/recognition/recognize"
# Face plugins are optional: "age,gender,landmarks,mask,calculator" etc.
FACE_PLUGINS = "landmarks"     # voeg "age,gender" toe indien gewenst
DET_PROB_THRESHOLD = 0.6       # minimum face confidence
PREDICTION_COUNT = 1           # top-N subject matches per face
SEND_EVERY_N_FRAMES = 3        # stuur slechts elke N frames naar de server
TIMEOUT_S = 5                  # HTTP-timeout

# === HELPERS ===
def recognize_frame_jpeg(jpeg_bytes: bytes) -> Dict[str, Any]:
    """POST een JPEG naar CompreFace en geef JSON terug."""
    params = {
        "prediction_count": PREDICTION_COUNT,
        "det_prob_threshold": DET_PROB_THRESHOLD,
        "face_plugins": FACE_PLUGINS,
        "status": "false",
        "detect_faces": "true",
    }
    headers = {"x-api-key": API_KEY}
    files = {"file": ("frame.jpg", jpeg_bytes, "image/jpeg")}
    r = requests.post(ENDPOINT, headers=headers, files=files, params=params, timeout=TIMEOUT_S)
    r.raise_for_status()
    return r.json()

def draw_result(frame, result_json):
    """Teken bounding boxes, naam/similarity en landmarks op het frame."""
    res = result_json.get("result", [])
    for item in res:
        box = item.get("box") or {}
        x1, y1, x2, y2 = int(box.get("x_min", 0)), int(box.get("y_min", 0)), int(box.get("x_max", 0)), int(box.get("y_max", 0))
        # bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # best match label
        label = "Unknown"
        subjects = item.get("subjects") or []
        if subjects:
            best = subjects[0]
            label = f"{best.get('subject','?')} ({best.get('similarity',0):.2f})"

        # tekst background + text
        cv2.rectangle(frame, (x1, y1-22), (x1 + max(80, len(label)*9), y1), (0, 255, 0), cv2.FILLED)
        cv2.putText(frame, label, (x1+5, y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

        # landmarks (optioneel)
        for lm in item.get("landmarks") or []:
            if isinstance(lm, (list, tuple)) and len(lm) == 2:
                cv2.circle(frame, (int(lm[0]), int(lm[1])), 2, (255, 0, 0), -1)

def main():
    cap = cv2.VideoCapture(1)  # pas camera index aan indien nodig
    if not cap.isOpened():
        raise RuntimeError("Kon de camera niet openen (index 0).")

    cv2.namedWindow("CompreFace Live", cv2.WINDOW_NORMAL)
    frame_id = 0
    last_json = None
    last_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            # Alleen elke N frames versturen om bandbreedte/CPU te sparen
            if frame_id % SEND_EVERY_N_FRAMES == 0:
                # comprimeer naar JPEG
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if ok:
                    try:
                        t0 = time.time()
                        last_json = recognize_frame_jpeg(buf.tobytes())
                        last_time = (time.time() - t0) * 1000.0
                    except requests.RequestException as e:
                        # Toon fout kort in het beeld, ga door
                        last_json = {"error": str(e)}

            # Teken laatste resultaat (ook als dit van eerder frame was)
            overlay = frame.copy()
            if last_json and "result" in last_json:
                draw_result(overlay, last_json)
                info = f"Inference: {last_time:.0f} ms | faces: {len(last_json.get('result', []))}"
                cv2.putText(overlay, info, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2, cv2.LINE_AA)
            elif last_json and "error" in last_json:
                cv2.putText(overlay, f"API error: {last_json['error']}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2, cv2.LINE_AA)

            cv2.imshow("CompreFace Live", overlay)
            frame_id += 1

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):   # ESC of q
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
