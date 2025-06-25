import sys
import asyncio
import json
import websockets
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock

# ✅ Standaard Signaling Server (wordt enkel overschreven indien expliciet opgegeven)
SIGNALING_SERVER = "ws://192.168.0.74:9000"
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]

class Undocker(Node):
    def __init__(self):
        super().__init__('undock_client')
        self._client = ActionClient(self, Undock, '/undock')

    def send_undock(self):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Undock server niet beschikbaar.')
            return False

        goal_msg = Undock.Goal()
        self._send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, self._send_future)
        goal_handle = self._send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Undock goal werd geweigerd.')
            return False

        self.get_logger().info('✅ Undock goal werd geaccepteerd.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ Undock voltooid.')
        return True

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

def main():
    rclpy.init()
    undocker = Undocker()
    success = undocker.send_undock()
    undocker.destroy_node()
    rclpy.shutdown()

    if success:
        try:
            asyncio.run(receive_direction())
        except KeyboardInterrupt:
            print("⏹️ Afgesloten door gebruiker")

if __name__ == "__main__":
    main()
