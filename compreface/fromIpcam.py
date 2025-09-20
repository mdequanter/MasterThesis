# ipcam_recognizer.py
import os, sys, time, argparse, threading
import requests
import cv2
import numpy as np

def parse_args():
    p = argparse.ArgumentParser("IP camera → CompreFace recognizer")
    p.add_argument("--rtsp", required=True, help="RTSP/HTTP URL, e.g. rtsp://user:pass@host/stream")
    p.add_argument("--api-key", default=os.getenv("CFX_API_KEY"), required=False, help="CompreFace API key")
    p.add_argument("--url", default=os.getenv("CFX_URL", "http://localhost:8000"), help="CompreFace base URL")
    p.add_argument("--plugins", default=os.getenv("CFX_PLUGINS", "none"), help="face_plugins, e.g. age,gender or none")
    p.add_argument("--topk", type=int, default=int(os.getenv("CFX_TOPK", "1")), help="prediction_count")
    p.add_argument("--det-prob", type=float, default=float(os.getenv("CFX_DET_PROB", "0.6")), help="detector prob 0..1")
    p.add_argument("--interval-ms", type=int, default=500, help="send every N ms")
    p.add_argument("--timeout", type=int, default=int(os.getenv("CFX_TIMEOUT", "8")), help="HTTP timeout seconds")
    p.add_argument("--resize", default="", help="optional WxH resize before sending, e.g. 640x360")
    p.add_argument("--display", action="store_true", help="show annotated preview window")
    p.add_argument("--label-thresh", type=float, default=0.6, help="draw label as OK if similarity>=thresh")
    return p.parse_args()

class Recognizer:
    def __init__(self, base_url, api_key, plugins, topk, det_prob, timeout):
        self.rec_url = base_url.rstrip("/") + "/api/v1/recognition/recognize"
        self.headers = {"x-api-key": api_key}
        self.params = {
            "prediction_count": max(1, int(topk)),
            "det_prob_threshold": float(det_prob),
            "face_plugins": plugins,
            "status": "false",
            "detect_faces": "true",
        }
        self.timeout = int(timeout)

    def recognize(self, bgr_img):
        ok, buf = cv2.imencode(".jpg", bgr_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return {"faces": []}
        files = {"file": ("frame.jpg", buf.tobytes(), "image/jpeg")}
        r = requests.post(self.rec_url, headers=self.headers, params=self.params, files=files, timeout=self.timeout)
        r.raise_for_status()
        data = r.json()
        faces = []
        for item in (data.get("result") or []):
            box = item.get("box") or {}
            subs = item.get("subjects") or []
            subj, sim = "Unknown", 0.0
            if subs:
                best = subs[0]
                subj = best.get("subject") or "Unknown"
                try: sim = float(best.get("similarity", 0.0))
                except: sim = 0.0
            faces.append({
                "subject": subj,
                "similarity": sim,
                "box": {
                    "x_min": int(box.get("x_min", 0)),
                    "y_min": int(box.get("y_min", 0)),
                    "x_max": int(box.get("x_max", 0)),
                    "y_max": int(box.get("y_max", 0)),
                }
            })
        return {"faces": faces}

def draw_faces(img, faces, ok_thresh):
    h, w = img.shape[:2]
    for f in faces:
        b = f["box"]
        x1, y1, x2, y2 = b["x_min"], b["y_min"], b["x_max"], b["y_max"]
        x1 = max(0, min(w - 1, x1)); x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1)); y2 = max(0, min(h - 1, y2))
        sim = float(f["similarity"])
        color = (40, 200, 40) if sim >= ok_thresh else (40, 140, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f'{f["subject"]} {sim*100:.1f}%'
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        y_text = max(0, y1 - 8)
        cv2.rectangle(img, (x1, y_text - th - 6), (x1 + tw + 6, y_text + 3), color, -1)
        cv2.putText(img, label, (x1 + 3, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 1, cv2.LINE_AA)

def main():
    args = parse_args()
    if not args.api_key:
        print("Missing --api-key or CFX_API_KEY", file=sys.stderr); sys.exit(1)

    rec = Recognizer(args.url, args.api_key, args.plugins, args.topk, args.det_prob, args.timeout)

    # optional resize
    target_wh = None
    if args.resize:
        try:
            w, h = args.resize.lower().split("x")
            target_wh = (int(w), int(h))
        except:
            print("Invalid --resize, expected WxH", file=sys.stderr); sys.exit(2)

    cap = None
    last_connect = 0
    frame = None
    frame_lock = threading.Lock()
    stop_flag = False

    def grab_loop():
        nonlocal cap, last_connect, frame, stop_flag
        while not stop_flag:
            if cap is None or not cap.isOpened():
                now = time.time()
                if now - last_connect < 2:
                    time.sleep(0.5); continue
                print("Connecting to camera...")
                # Prefer FFmpeg backend for RTSP stability
                cap = cv2.VideoCapture(args.rtsp, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FPS, 25)
                last_connect = now
                if not cap.isOpened():
                    print("Open failed. Retry soon.")
                    time.sleep(1)
                    continue
            ok, f = cap.read()
            if not ok:
                print("Read failed. Reconnecting.")
                cap.release(); cap = None
                time.sleep(0.2)
                continue
            if target_wh:
                f = cv2.resize(f, target_wh, interpolation=cv2.INTER_AREA)
            with frame_lock:
                frame = f
            if not args.display:
                # Small sleep to avoid tight loop if not previewing
                time.sleep(0.001)

    def infer_loop():
        nonlocal stop_flag
        interval = max(50, args.interval_ms) / 1000.0
        while not stop_flag:
            t0 = time.time()
            img = None
            with frame_lock:
                if frame is not None:
                    img = frame.copy()
            if img is not None:
                try:
                    res = rec.recognize(img)
                    faces = res.get("faces", [])
                    draw_faces(img, faces, args.label_thresh)
                    if args.display:
                        cv2.imshow("IPCam Recognition", img)
                        if cv2.waitKey(1) & 0xFF == 27:  # ESC
                            break
                except requests.RequestException as e:
                    print(f"HTTP error: {e}")
                except Exception as e:
                    print(f"Error: {e}")
            dt = time.time() - t0
            sleep = max(0.0, interval - dt)
            time.sleep(sleep)
        stop_flag = True

    t_grab = threading.Thread(target=grab_loop, daemon=True)
    t_grab.start()
    try:
        infer_loop()
    finally:
        stop_flag = True
        t_grab.join(timeout=1.0)
        try:
            if cap: cap.release()
        except: pass
        if args.display:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
