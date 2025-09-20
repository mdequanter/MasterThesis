#!/usr/bin/env python3
# send_direction.py
# Vereist: pip install websockets

import asyncio
import json
import time
import argparse
import sys
import websockets

def clamp_angle(v: float) -> float:
    return max(0.0, min(180.0, float(v)))

async def send_direction(url: str, angle: float, count: int, rate_hz: float):
    angle = clamp_angle(angle)
    delay = 1.0 / rate_hz if rate_hz > 0 else 0.0

    try:
        async with websockets.connect(url) as ws:
            for i in range(count):
                payload = {
                    "direction_angle": angle+i,
                    "ts": time.time()
                }
                msg = json.dumps(payload, separators=(',', ':'))
                await ws.send(msg)
                print(f"sent {i+1}/{count}: {msg}")
                if i + 1 < count and delay > 0:
                    await asyncio.sleep(delay)
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)

def main():
    p = argparse.ArgumentParser(description="Stuur direction_angle naar WebSocket.")
    p.add_argument("--url", default="ws://192.168.0.74:9000", help="WebSocket URL")
    p.add_argument("--angle", type=float, default=90, help="hoek 0..180")
    p.add_argument("--count", type=int, default=1, help="aantal berichten")
    p.add_argument("--rate", type=float, default=2.0, help="berichten per seconde bij count>1")
    args = p.parse_args()
    asyncio.run(send_direction(args.url, args.angle, args.count, args.rate))

if __name__ == "__main__":
    main()
