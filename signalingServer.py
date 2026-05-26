import asyncio
import websockets

clients = set()

async def signaling(websocket):
    clients.add(websocket)
    print(f"✅ Nieuwe client verbonden: {websocket.remote_address}")
    
    try:
        async for message in websocket:
            print(f"📩 Bericht ontvangen van {websocket.remote_address}: {message}")
            for client in clients:
                if client != websocket:
                    await client.send(message)
                    #print(f"📤 Bericht doorgestuurd naar {client.remote_address}")
    except websockets.exceptions.ConnectionClosedError:
        print(f"⚠ Client {websocket.remote_address} heeft de verbinding verbroken.")
    finally:
        clients.remove(websocket)
        print(f"❌ Client verwijderd: {websocket.remote_address}")

async def start_server():
    print("🚀 WebSocket Signaling Server wordt gestart op ws://0.0.0.0:9000")
    async with websockets.serve(signaling, "0.0.0.0", 9000): 
        await asyncio.Future()  # Houd de server actief

if __name__ == "__main__":
    asyncio.run(start_server())
