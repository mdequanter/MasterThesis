import sys
import asyncio
import json
import websockets
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
import time

SIGNALING_SERVER = "ws://192.168.0.74:9000"
COMMAND_RATE = 2
MAX_ANGULAR = 10.0  # Max angular speed in rad/s

# ✅ Parse CLI arguments
for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]
    elif arg.startswith("COMMAND_RATE="):
        try:
            COMMAND_RATE = float(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige COMMAND_RATE, standaard blijft:", COMMAND_RATE)
    elif arg.startswith("MAX_ANGULAR="):
        try:
            MAX_ANGULAR = float(arg.split("=")[1])
        except ValueError:
            print("⚠️ Ongeldige MAX_ANGULAR waarde, standaard blijft:", MAX_ANGULAR)

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

class DirectionController(Node):
    def __init__(self):
        super().__init__('direction_controller')
        self.publisher: Publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buffer = []
        self.last_publish_time = time.time()

    def add_direction(self, angle):
        self.buffer.append(angle)

    def process(self):
        now = time.time()
        if now - self.last_publish_time >= 1.0 / COMMAND_RATE:
            if not self.buffer:
                return
            avg_angle = sum(self.buffer) / len(self.buffer)
            self.buffer.clear()

            twist = Twist()
            twist.linear.x = 0.1  # 🚫 niet vooruit
            error = avg_angle - 90.0



            if abs(error) < 1.0:
                twist.angular.z = 0.0
                twist.linear.x = 0.2  # 🚫 niet vooruit
            else:
                twist.linear.x = 0.05
                proportion = error / 90.0
                twist.angular.z = (max(-MAX_ANGULAR, min(MAX_ANGULAR, proportion * MAX_ANGULAR)))*1.00
                
            if (round(avg_angle,2) == 90.00):  # er is geen detectie
                twist.angular.z = 0.5
                twist.linear.x = 0.0  # 🚫 niet vooruit



            self.publisher.publish(twist)
            print(f"➡️ Gemiddelde richting: {avg_angle:.2f}° → angular.z = {twist.angular.z:.2f}")
            self.last_publish_time = now

async def receive_direction(controller: DirectionController):
    print(f"📡 Verbinden met: {SIGNALING_SERVER}")
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        while rclpy.ok():
            try:
                message = await websocket.recv()
                data = json.loads(message)
                if "direction_angle" in data:
                    controller.add_direction(data["direction_angle"])
                controller.process()
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

# ✅ Start ROS2 node en asyncio WebSocket loop
    controller = DirectionController()

    loop = asyncio.get_event_loop()
    loop.create_task(receive_direction(controller))  # async direction handler

    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            loop.run_until_complete(asyncio.sleep(0.01))  # laat asyncio ook draaien
    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

        
if __name__ == "__main__":
    main()
