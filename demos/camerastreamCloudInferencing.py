import asyncio, json, time, base64, cv2, websockets, os, sys
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend

RTSP_URL = "rtsp://admin:Nijverheidskaai170@10.2.172.109"
SIGNALING_SERVER = "ws://94.111.36.87:9000"
AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '
JPEG_QUALITY = 45
MAX_FPS = 8
DISPLAY = ("--display" in sys.argv)

for a in sys.argv[1:]:
    if a.startswith("RTSP="): RTSP_URL = a.split("=",1)[1]
    elif a.startswith("SIGNALING_SERVER="): SIGNALING_SERVER = a.split("=",1)[1]
    elif a.startswith("JPEG_QUALITY="): JPEG_QUALITY = int(a.split("=",1)[1])
    elif a.startswith("MAX_FPS="): MAX_FPS = int(a.split("=",1)[1])

def encrypt_bytes(b: bytes) -> str:
    iv = os.urandom(16)
    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(iv), backend=default_backend())
    enc = cipher.encryptor()
    padder = padding.PKCS7(algorithms.AES.block_size).padder()
    padded = padder.update(b) + padder.finalize()
    data = enc.update(padded) + enc.finalize()
    return base64.b64encode(iv + data).decode("utf-8")

async def send_once():
    # Disable keepalive pings to avoid ping timeouts on heavy streams
    async with websockets.connect(
        SIGNALING_SERVER,
        max_size=None,
        ping_interval=None,
        close_timeout=1,
    ) as ws:
        print("Connected")
        cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print("RTSP open failed"); return
        delay = 1.0 / MAX_FPS if MAX_FPS > 0 else 0
        frame_id = 0
        while True:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok or frame is None:
                await asyncio.sleep(0.05); continue

            # Optional downscale to reduce bandwidth
            h, w = frame.shape[:2]
            if max(h, w) > 720:
                frame = cv2.resize(frame, (int(w*0.5), int(h*0.5)))

            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok: continue
            enc = encrypt_bytes(jpeg.tobytes())
            frame_id += 1
            msg = {"frame_id": frame_id, "data": enc, "ts": time.time()}
            await ws.send(json.dumps(msg))

            if DISPLAY:
                cv2.imshow("RTSP Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if delay:
                await asyncio.sleep(max(0.0, delay - (time.time() - t0)))

        cap.release()
        if DISPLAY: cv2.destroyAllWindows()

async def main():
    # Auto-reconnect on close/timeouts
    while True:
        try:
            await send_once()
            break
        except (websockets.exceptions.ConnectionClosedError,
                websockets.exceptions.ConnectionClosedOK,
                asyncio.TimeoutError) as e:
            print(f"WS error: {e}. Reconnecting in 2s.")
            await asyncio.sleep(2)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
