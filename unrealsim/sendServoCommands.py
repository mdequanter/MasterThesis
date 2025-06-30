#!/usr/bin/env python3
import asyncio
import websockets
import json
import time
from collections import deque
import serial
import sys

# Configuratie
SIGNALING_SERVER_DIRECTION = "ws://192.168.0.74:9000"
SERIAL_PORT = "/dev/ttyUSB0"   # Pas dit aan indien nodig
BAUDRATE = 115200

# Buffer voor de laatste seconde
direction_history = deque()
latest_direction = None

async def get_direction():
    global latest_direction
    print(f"📡 Verbinden met {SIGNALING_SERVER_DIRECTION}")
    async with websockets.connect(SIGNALING_SERVER_DIRECTION) as websocket:
        print("✅ Verbonden met direction server")
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                direction = data.get("direction_angle")
                timestamp = time.time()
                if direction is not None:
                    # Buffer bijwerken
                    direction_history.append((timestamp, direction))

                    # Oude waarden verwijderen (>1 sec)
                    while direction_history and direction_history[0][0] < timestamp - 1.0:
                        direction_history.popleft()

                    # Gemiddelde berekenen
                    if direction_history:
                        values = [v for t, v in direction_history]
                        avg_direction = sum(values) / len(values)
                        latest_direction = avg_direction
                    else:
                        latest_direction = None

            except Exception as e:
                print(f"⚠️ WebSocket error: {e}")
                await asyncio.sleep(1)

async def main():
    global latest_direction

    # Seriële verbinding openen
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # Even wachten tot WEMOS klaar is
        print(f"✅ Seriële verbinding geopend op {SERIAL_PORT}")
    except Exception as e:
        print(f"❌ Kon seriële poort niet openen: {e}")
        sys.exit(1)

    # Start WebSocket task
    direction_task = asyncio.create_task(get_direction())

    try:
        while True:
            if latest_direction is not None:
                angle_int = int(max(0, min(180, latest_direction)))
                ser.write(f"{angle_int}\n".encode())
                print(f"➡️ Servo angle gestuurd: {angle_int}")

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")

    finally:
        ser.close()
        print("🔌 Seriële verbinding gesloten.")

if __name__ == "__main__":
    asyncio.run(main())
