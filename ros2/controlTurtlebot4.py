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
import threading
import cv2

SIGNALING_SERVER = "ws://192.168.0.74:9000"
COMMAND_RATE = 2
MAX_ANGULAR = 10.0  # Max angular speed in rad/s

screenOutput = True
fullscreen = False
CAMERA_INDEX = 0

linear_speed = 0.0
angular_speed = 0.0
align_with_arrow = False
latest_direction_angle = 90.0  # Default richting vooruit

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
        self.last_publish_time = time.time()

    def publish_manual_control(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)
        print(f"🕹️ Published manual control: linear.x = {linear_x:.2f}, angular.z = {angular_z:.2f}")

    def align_to_direction(self, angle):
        error = angle - 90.0
        proportion = error / 90.0
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, proportion * MAX_ANGULAR))
        self.publish_manual_control(0.0, angular_z)

async def receive_direction(controller: DirectionController):
    global latest_direction_angle
    print(f"📡 Verbinden met: {SIGNALING_SERVER}")
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        while rclpy.ok():
            try:
                message = await websocket.recv()
                data = json.loads(message)
                if "direction_angle" in data:
                    latest_direction_angle = data["direction_angle"]
            except Exception as e:
                print(f"⚠️ Fout bij verwerken bericht: {e}")
                break

def keyboard_loop(controller: DirectionController):
    global linear_speed, angular_speed, screenOutput, fullscreen, align_with_arrow
    cap = None

    if screenOutput:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if fullscreen:
            cv2.namedWindow("Camera", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty("Camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while True:
        key = input("Toets (z/s/w/q/e/d/v/f/a): ").strip().lower()
        if key == 'z':
            linear_speed += 0.05
        elif key == 'w':
            linear_speed -= 0.05
        elif key == 's':
            linear_speed = 0.0
            angular_speed = 0.0
        elif key == 'q':
            angular_speed += 0.1
        elif key == 'e':
            angular_speed -= 0.1
        elif key == 'd':
            checker = DockChecker()
            is_docked = checker.is_docked()
            checker.destroy_node()
            if is_docked:
                undocker = Undocker()
                undocker.send_undock()
                undocker.destroy_node()
            else:
                print("🛑 Reeds ontkoppeld.")
        elif key == 'v':
            screenOutput = not screenOutput
            if not screenOutput and cap:
                cap.release()
                cv2.destroyAllWindows()
        elif key == 'f':
            fullscreen = not fullscreen
            if screenOutput:
                cv2.setWindowProperty("Camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
        elif key == 'a':
            align_with_arrow = True
        if align_with_arrow:
            controller.align_to_direction(latest_direction_angle)
            align_with_arrow = False
        else:
            controller.publish_manual_control(linear_speed, angular_speed)

        if screenOutput and cap:
            ret, frame = cap.read()
            if ret:
                cv2.imshow("Camera", frame)
                cv2.waitKey(1)


def main():
    rclpy.init()
    controller = DirectionController()
    loop = asyncio.get_event_loop()
    loop.create_task(receive_direction(controller))

    t = threading.Thread(target=keyboard_loop, args=(controller,), daemon=True)
    t.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            loop.run_until_complete(asyncio.sleep(0.01))
    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
