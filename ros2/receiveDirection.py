import sys
import asyncio
import json
import websockets
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
import time

SIGNALING_SERVER = "ws://192.168.0.74:9000"
COMMAND_RATE = 5  # keer per seconde een gemiddelde printen

# ✅ Commandline parameters verwerken
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]
    elif arg.startswith("COMMAND_RATE="):
        try:
            COMMAND_RATE = float(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige COMMAND_RATE, standaard blijft:", COMMAND_RATE)

class DockChecker(Node):
    def __init__(self):
        super().__init__('dock_checker')
        self.dock_status = None
        self.sub = self.create_subscription(
            DockStatus,
            '/dock_status',
            self.callback,
            qos_profile_sensor_data
        )
        self.received = False

    def callback(self, msg):
        self.dock_status = msg
        self.received = True

    def is_docked(self, timeout_sec=3.0):
        start_time = self.get_clock().now()
        while not self.received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warn("⚠️ Geen /dock_status ontvangen binnen timeout.")
                return False
        return self.dock_status.is_docked

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
            self.get_logger().error('❌ Undock goal geweigerd.')
            return False

        self.get_logger().info('✅ Undock goal geaccepteerd.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ Undock voltooid.')
        return True

async def receive_direction():
    print(f"📡 Luisteren op: {SIGNALING_SERVER}")
    direction_buffer = []
    last_print_time = time.time()

    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)

                if "direction_angle" in data:
                    direction = data["direction_angle"]
                    direction_buffer.append(direction)

                current_time = time.time()
                if current_time - last_print_time >= 1.0 / COMMAND_RATE:
                    if direction_buffer:
                        avg = sum(direction_buffer) / len(direction_buffer)
                        print(f"➡️ Gemiddelde richting (laatste {len(direction_buffer)}): {avg:.2f}°")
                        direction_buffer.clear()
                    last_print_time = current_time

            except websockets.exceptions.ConnectionClosed:
                print("❌ Verbinding verbroken")
                break
            except Exception as e:
                print(f"⚠️ Fout bij verwerken bericht: {e}")

def main():
    rclpy.init()

    checker = DockChecker()
    is_docked = checker.is_docked()
    checker.destroy_node()

    if is_docked:
        print("📦 Robot is gedockt, probeer te undocken...")
        undocker = Undocker()
        success = undocker.send_undock()
        undocker.destroy_node()
        if not success:
            rclpy.shutdown()
            return
    else:
        print("✅ Robot is NIET gedockt. Geen undock nodig.")

    rclpy.shutdown()

    try:
        asyncio.run(receive_direction())
    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")

if __name__ == "__main__":
    main()
