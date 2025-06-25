import sys
import asyncio
import json
import websockets

# ✅ Signaling server instellen via commandline
SIGNALING_SERVER = "ws://localhost:9000"
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]

async def receive_direction():
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                if "direction_angle" in data:
                    print(f"➡️ Direction angle: {data['direction_angle']}°")
            except websockets.exceptions.ConnectionClosed:
                print("❌ Verbinding verbroken")
                break
            except Exception as e:
                print(f"⚠️ Fout bij verwerken bericht: {e}")

try:
    asyncio.run(receive_direction())
except KeyboardInterrupt:
    print("⏹️ Afgesloten door gebruiker")
