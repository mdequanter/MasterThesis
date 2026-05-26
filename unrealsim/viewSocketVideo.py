import sys
import asyncio
import json
import websockets
import cv2
import numpy as np
import base64
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend

# --- Settings ---
SIGNALING_SERVER = "ws://94.111.36.87:9000"
AES_KEY = b'C\x03\xb6\xd2\xc5\t.Brp\x1ce\x0e\xa4\xf6\x8b\xd2\xf6\xb0\x8a\x9c\xd5D\x1e\xf4\xeb\x1d\xe6\x0c\x1d\xff '

# Optional CLI overrides: SIGNALING_SERVER=ws://host:port
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]

print(f"Signaling Server: {SIGNALING_SERVER}")

def decrypt_data(encrypted_base64: str) -> bytes:
    encrypted_data = base64.b64decode(encrypted_base64)
    iv = encrypted_data[:16]
    encrypted_bytes = encrypted_data[16:]
    cipher = Cipher(algorithms.AES(AES_KEY), modes.CBC(iv), backend=default_backend())
    decryptor = cipher.decryptor()
    decrypted_padded = decryptor.update(encrypted_bytes) + decryptor.finalize()
    unpadder = padding.PKCS7(algorithms.AES.block_size).unpadder()
    return unpadder.update(decrypted_padded) + unpadder.finalize()

async def receive_and_show():
    async with websockets.connect(SIGNALING_SERVER) as ws:
        print("Connected.")
        while True:
            try:
                msg = await ws.recv()
            except websockets.exceptions.ConnectionClosed:
                print("Connection closed.")
                break

            # Expect JSON with {"data": "<base64_aes_cbc_jpeg>", ...}
            try:
                payload = json.loads(msg)
                enc_b64 = payload.get("data")
                if not enc_b64:
                    continue
            except json.JSONDecodeError:
                continue

            try:
                jpeg_bytes = decrypt_data(enc_b64)
                np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue
            except Exception as e:
                print(f"Decode error: {e}")
                continue

            cv2.imshow("WebSocket Video", frame)
            # Press q to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(receive_and_show())
