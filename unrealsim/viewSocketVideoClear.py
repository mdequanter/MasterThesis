import sys
import asyncio
import json
import websockets
import cv2
import numpy as np
import base64

# --- Settings ---
SIGNALING_SERVER = "ws://192.168.0.74:9000"

# Optional CLI overrides: SIGNALING_SERVER=ws://host:port
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]

print(f"Signaling Server: {SIGNALING_SERVER}")

def decode_message(msg):
    """
    msg kan bytes (raw JPEG) of str (JSON met base64 JPEG) zijn.
    Retourneert bytes van een JPEG of None.
    """
    if isinstance(msg, (bytes, bytearray)):
        return bytes(msg)

    if isinstance(msg, str):
        # Probeer JSON met "data": "<base64>"
        try:
            payload = json.loads(msg)
            b64 = payload.get("data")
            if not b64:
                return None
            return base64.b64decode(b64)
        except json.JSONDecodeError:
            return None

    return None

async def receive_and_show():
    # max_size=None om grote frames toe te laten
    async with websockets.connect(SIGNALING_SERVER, max_size=None) as ws:
        print("Connected.")
        while True:
            try:
                msg = await ws.recv()
            except websockets.exceptions.ConnectionClosed:
                print("Connection closed.")
                break

            jpeg_bytes = decode_message(msg)
            if not jpeg_bytes:
                continue

            np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            cv2.imshow("WebSocket Video (unencrypted)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_and_show())
