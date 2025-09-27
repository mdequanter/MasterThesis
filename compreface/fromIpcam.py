# ipcam_motion_recognizer.py
import os, sys, time, argparse, threading, queue, datetime
import requests
import cv2
import numpy as np

def parse_args():
    p = argparse.ArgumentParser("IP camera → motion-triggered CompreFace recognizer")
    p.add_argument("--rtsp", required=True, help="RTSP/HTTP URL, e.g. rtsp://user:pass@host/stream")
    p.add_argument("--api-key", default=os.getenv("CFX_API_KEY"), help="CompreFace API key")
    p.add_argument("--url", default=os.getenv("CFX_URL", "http://localhost:8000"), help="CompreFace base URL")
    p.add_argument("--plugins", default=os.getenv("CFX_PLUGINS", ""), help="comma list: age,gender,landmarks,mask")
    p.add_argument("--topk", type=int, default=int(os.getenv("CFX_TOPK", "1")), help="prediction_count/limit")
    p.add_argument("--det-prob", type=float, default=float(os.getenv("CFX_DET_PROB", "0.6")), help="detector prob 0..1")
    p.add_argument("--timeout", type=int, default=int(os.getenv("CFX_TIMEOUT", "8")), help="HTTP timeout seconds")
    p.add_argument("--resize", default="", help="optional WxH before processing, e.g. 640x360")
    p.add_argument("--display", action="store_true", help="show live window")
    # Motion settings
    p.add_argument("--motion-ratio", type=float, default=0.002, help="ratio of moving pixels to trigger (0..1)")
    p.add_argument("--motion-min-px", type=int, default=1500, help="absolute pixel threshold floor")
    p.add_argument("--cooldown-s", type=float, default=2.0, help="min seconds between triggers")
    # Saving
    p.add_argument("--save-dir", default="captures", help="folder for printscreens and results")
    p.add_argument("--jpeg-quality", type=int, default=90, help="JPEG quality for saved images")
    p.add_argument("--label-thresh", type=float, default=0.6, help="OK color if similarity>=thresh")
    return p.parse_args()

class Recognizer:
    def __init__(self, base_url, api_key, plugins, topk, det_prob, timeout):
        if not api_key:
            print("Missing --api-key or CFX_API_KEY", file=sys.stderr); sys.exit(1)
        self.rec_url = base_url.rstrip("/") + "/api/v1/recognition/recognize"
        self.headers = {"x-api-key": api_key}
        self.params = {
            "limit": max(1, int(topk)),
            "prediction_count": max(1, int(topk)),
            "det_prob_threshold": float(det_prob),
        }
        if plugins:
            self.params["face_plugins"] = plugins
        self.timeout = int(timeout)

    def recognize(self, bgr_img):
        ok, buf = cv2.imencode(".jpg", bgr_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return {"faces": []}
        files = {"file": ("frame.jpg", buf.tobytes(), "image/jpeg")}
        r = requests.post(self.rec_url, headers=self.headers, params=self.params, files=files, timeout=self.timeout)
        try:
            r.raise_for_status()
        except requests.HTTPError:
            print(f"HTTP {r.status_code}: {r.text[:300]}")
            raise
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

def timestamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")

def main():
    args = parse_args()
    os.makedirs(args.save_dir, exist_ok=True)

    # Camera
    cap = cv2.VideoCapture(args.rtsp, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FPS, 25)
    if not cap.isOpened():
        print("Failed to open camera", file=sys.stderr); sys.exit(3)

    # Optional resize
    target_wh = 800, 600
    if args.resize:
        try:
            w, h = args.resize.lower().split("x")
            target_wh = (int(w), int(h))
        except:
            print("Invalid --resize, expected WxH", file=sys.stderr); sys.exit(2)

    # Background subtractor
    bg = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=16, detectShadows=True)

    # Motion → recognition queue
    q = queue.Queue(maxsize=2)
    rec = Recognizer(args.url, args.api_key, args.plugins, args.topk, args.det_prob, args.timeout)

    last_trigger = 0.0
    jpeg_q = int(np.clip(args.jpeg_quality, 50, 100))

    # Worker for recognition only when motion frames arrive
    def worker():
        while True:
            item = q.get()
            if item is None:
                break
            raw_img, path_base = item
            try:
                # Save raw printscreen
                raw_path = f"{path_base}.jpg"
                cv2.imencode(".jpg", raw_img, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_q])[1].tofile(raw_path)

                # Face detection
                # inside the motion trigger when faces are detected
                res = rec.recognize(frame)
                faces = res.get("faces", [])

                if faces:
                    subj = faces[0]["subject"].replace(" ", "_")
                    sim  = f"{faces[0]['similarity']:.2f}"
                else:
                    subj = "Unknown"
                    sim  = "0.00"

                fname_base = f"{subj}_{sim}_{timestamp()}"
                raw_path = os.path.join(args.save_dir, f"{fname_base}.jpg")
                ann_path = os.path.join(args.save_dir, f"{fname_base}_det.jpg")

                cv2.imwrite(raw_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
                out = frame.copy()
                if faces:
                    draw_faces(out, faces, args.label_thresh)
                cv2.imwrite(ann_path, out, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])

                print(f"Saved {raw_path}, {len(faces)} faces")

            except Exception as e:
                print(f"Worker error: {e}")
            finally:
                q.task_done()

    t = threading.Thread(target=worker, daemon=True)
    t.start()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Read failed, retrying...")
                time.sleep(0.2)
                continue

            if target_wh:
                frame = cv2.resize(frame, target_wh, interpolation=cv2.INTER_AREA)

            # Motion detection
            fg = bg.apply(frame)
            # Remove shadows and noise
            _, fg_bin = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
            fg_bin = cv2.morphologyEx(fg_bin, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)

            moving_px = int(cv2.countNonZero(fg_bin))
            h, w = frame.shape[:2]
            dyn_thresh = max(args.motion_min_px, int(args.motion_ratio * w * h))
            motion = moving_px >= dyn_thresh

            # Trigger on motion with cooldown
            now = time.time()
            if motion and (now - last_trigger) >= args.cooldown_s:
                last_trigger = now
                base = os.path.join(args.save_dir, f"{timestamp()}")
                # Push a copy so UI remains smooth
                try:
                    if not q.full():
                        q.put_nowait((frame.copy(), base))
                    else:
                        # Drop if queue busy
                        pass
                except queue.Full:
                    pass

            if args.display:
                # Small HUD
                disp = frame.copy()
                if motion:
                    cv2.putText(disp, "MOTION", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2, cv2.LINE_AA)
                cv2.putText(disp, f"px:{moving_px} thr:{dyn_thresh}", (10, 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("IPCam Motion+Recognition (on trigger)", disp)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break
            else:
                # Yield a bit
                time.sleep(0.001)

    finally:
        try:
            cap.release()
        except:
            pass
        if args.display:
            cv2.destroyAllWindows()
        # stop worker
        q.put(None)
        t.join(timeout=1.0)

if __name__ == "__main__":
    main()
