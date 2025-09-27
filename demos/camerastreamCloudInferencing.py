import asyncio, json, time, base64, cv2, websockets, os, sys
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend

RTSP_URL = "rtsp://admin:Nijverheidskaai170@10.2.172.109"
SIGNALING_SERVER = "ws://192.168.0.74:9000"
AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '
JPEG_QUALITY = 50
MAX_FPS = 10
DISPLAY = False

# Optional CLI overrides
for arg in sys.argv[1:]:
    if arg.startswith("RTSP="): RTSP_URL = arg.split("=",1)[1]
    elif arg.startswith("SIGNALING_SERVER="): SIGNALING_SERVER = arg.split("=",1)[1]
    elif arg.startswith("JPEG_QUALITY="): JPEG_QUALITY = int(arg.split("=",1)[1])
    elif arg.startswith("MAX_FPS="): MAX_FPS = int(arg.split("=",1)[1])
    elif arg == "--display": DISPLAY = True

def encrypt_bytes(b: bytes) -> str:
    iv = os.urandom(16)
    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(iv), backend=default_backend())
    enc = cipher.encryptor()
    padder = padding.PKCS7(algorithms.AES.block_size).padder()
    padded = padder.update(b) + padder.finalize()
    data = enc.update(padded) + enc.finalize()
    return base64.b64encode(iv + data).decode("utf-8")

async def send_stream():
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("RTSP open failed"); return
    frame_id = 0
    delay = 1.0 / MAX_FPS if MAX_FPS > 0 else 0
    async with websockets.connect(SIGNALING_SERVER, max_size=None) as ws:
        print("Connected")
        while True:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok or frame is None:
                await asyncio.sleep(0.05); continue

            # Compress
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok: continue

            # Encrypt
            enc = encrypt_bytes(jpeg.tobytes())
            frame_id += 1
            msg = {"frame_id": frame_id, "data": enc, "timestamp": time.time()}
            await ws.send(json.dumps(msg))

            # Show locally
            if DISPLAY:
                cv2.imshow("RTSP Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if delay:
                await asyncio.sleep(max(0.0, delay - (time.time() - t0)))

    cap.release()
    if DISPLAY: cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        asyncio.run(send_stream())
    except KeyboardInterrupt:
        pass
