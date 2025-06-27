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
from rclpy.task import Future
from std_msgs.msg import String  # Als placeholder voor IR-data (pas aan naar jouw type)

import time
import threading
import tty
import termios
import select
import queue

# ✅ Standaardinstellingen
SIGNALING_SERVER = "ws://192.168.0.74:9000"
COMMAND_RATE = 2
MAX_ANGULAR = 3.0
NO_DETECTION_TIMEOUT = 0.5
OBSTACLE_THRESHOLD = 20  # Grootte waarboven obstakel

linear_speed = 0.0
angular_speed = 0.0
align_with_arrow = False
latest_direction_angle = 90.0
collision_avoidance = True  # 🚧 standaard actief

command_queue = queue.Queue()

for arg in sys.argv[1:]:
    if arg.startswith("SIGNALING_SERVER="):
        SIGNALING_SERVER = arg.split("=", 1)[1]
    elif arg.startswith("COMMAND_RATE="):
        COMMAND_RATE = float(arg.split("=")[1])
    elif arg.startswith("MAX_ANGULAR="):
        MAX_ANGULAR = float(arg.split("=")[1])
    elif arg.startswith("NO_DETECTION_TIMEOUT="):
        NO_DETECTION_TIMEOUT = float(arg.split("=")[1])

class DockChecker(Node):
    def __init__(self):
        super().__init__('dock_checker')
        self.dock_status = None
        self.received = False
        self.sub = self.create_subscription(
            DockStatus,
            '/dock_status',
            self.callback,
            qos_profile_sensor_data
        )

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
        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
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
        self.ir_subscription = self.create_subscription(
            String,  # Pas aan naar je echte msg type
            '/ir_intensity',
            self.ir_callback,
            qos_profile_sensor_data
        )
        self.last_ir_msg = None

    def ir_callback(self, msg):
        self.last_ir_msg = msg

    def publish_manual_control(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)

    def align_to_direction(self, linear_x, angle):
        error = angle - 90.0
        proportion = error / 90.0
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, proportion * MAX_ANGULAR))
        self.publish_manual_control(linear_x, angular_z)

    def avoid_collision(self):
        if not self.last_ir_msg:
            return False
        # Parsing: hier moet je je echte message fields gebruiken
        msg_dict = json.loads(self.last_ir_msg.data)
        readings = msg_dict["readings"]

        obstacle_left = False
        obstacle_right = False
        obstacle_front = False

        for r in readings:
            value = r["value"]
            frame_id = r["header"]["frame_id"]

            if value > OBSTACLE_THRESHOLD:  # ✅ Groot = obstakel
                if "left" in frame_id:
                    obstacle_left = True
                elif "right" in frame_id:
                    obstacle_right = True
                elif "center" in frame_id:
                    obstacle_front = True

        if obstacle_front:
            print("🚧 Obstacle front → Stop")
            self.publish_manual_control(0.0, 0.0)
            return True
        elif obstacle_left:
            print("↪️ Obstacle left → Turn right")
            self.publish_manual_control(0.0, -0.5)
            return True
        elif obstacle_right:
            print("↩️ Obstacle right → Turn left")
            self.publish_manual_control(0.0, 0.5)
            return True

        return False

async def receive_direction(controller: DirectionController):
    global latest_direction_angle, align_with_arrow, collision_avoidance

    last_detection_time = time.time()
    print(f"📡 Verbinden met: {SIGNALING_SERVER}")
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print(f"✅ Verbonden met {SIGNALING_SERVER}")
        while rclpy.ok():
            try:
                message = await websocket.recv()
                data = json.loads(message)

                if "direction_angle" in data:
                    latest_direction_angle = data["direction_angle"]

                if "detected" in data:
                    detected = data["detected"]
                    current_time = time.time()

                    if detected:
                        last_detection_time = current_time
                    else:
                        if (current_time - last_detection_time > NO_DETECTION_TIMEOUT):
                            print(f"🚫 Geen detectie > {NO_DETECTION_TIMEOUT}s → Collision avoidance AAN")
                            collision_avoidance = True
                            align_with_arrow = False
            except Exception as e:
                print(f"⚠️ Fout bij verwerken bericht: {e}")
                break

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        [i, _, _] = select.select([sys.stdin], [], [], 0.1)
        if i:
            key = sys.stdin.read(1)
        else:
            key = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def keyboard_loop():
    global linear_speed, angular_speed, align_with_arrow, collision_avoidance

    while True:
        key = get_key()
        if key:
            if key == 'z':
                linear_speed += 0.1
            elif key == 'w':
                linear_speed -= 0.1
            elif key == 's':
                linear_speed = 0.0
                angular_speed = 0.0
            elif key == 'a':
                angular_speed += 0.1
            elif key == 'e':
                angular_speed -= 0.1
            elif key == 'd':
                command_queue.put('undock')
            elif key == 'c':
                align_with_arrow = not align_with_arrow
                print(f"🔄 Align with arrow {'ingeschakeld' if align_with_arrow else 'uitgeschakeld'}")
            elif key == 'o':
                collision_avoidance = not collision_avoidance
                print(f"🚧 Collision avoidance {'ingeschakeld' if collision_avoidance else 'uitgeschakeld'}")
            elif key == 'q':
                print("⏹️ Afsluiten op verzoek van gebruiker (q)")
                sys.exit(0)
        time.sleep(0.1)

def main():
    global align_with_arrow, collision_avoidance
    rclpy.init()
    controller = DirectionController()
    loop = asyncio.get_event_loop()
    loop.create_task(receive_direction(controller))

    t = threading.Thread(target=keyboard_loop, daemon=True)
    t.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)

            if collision_avoidance:
                if controller.avoid_collision():
                    continue

            if align_with_arrow:
                controller.align_to_direction(linear_speed, latest_direction_angle)
            else:
                controller.publish_manual_control(linear_speed, angular_speed)

            try:
                cmd = command_queue.get_nowait()
                if cmd == 'undock':
                    checker = DockChecker()
                    if checker.is_docked():
                        undocker = Undocker()
                        undocker.send_undock()
                        undocker.destroy_node()
                    else:
                        print("🛑 Reeds ontkoppeld.")
            except queue.Empty:
                pass

            loop.run_until_complete(asyncio.sleep(0.01))
    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
