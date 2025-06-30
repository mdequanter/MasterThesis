#!/usr/bin/env python3

import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import HazardDetectionVector
from sensor_msgs.msg import BatteryState
import sys
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
import time
from collections import deque
import serial

# Configuratie
SIGNALING_SERVER_DIRECTION = "ws://192.168.0.74:9000"
SERIAL_PORT = "/dev/ttyUSB0"   # Pas aan indien nodig
BAUDRATE = 115200

MAX_ANGULAR = 1
OB_TRESHOLD = 50
MIN_BATTERY_LEVEL_PCT = 0.2
forward_speed = 0.1
turning_speed = 0.5

latest_direction = None
latest_detected = None
latest_ir_data = None
latest_hazard_data = None
latest_battery_state = None

# Buffer voor direction
direction_history = deque()


class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, qos_profile)
        self.get_logger().info("✅ Subscribed to /battery_state")

    def battery_callback(self, msg):
        global latest_battery_state
        latest_battery_state = msg.percentage


class IRListener(Node):
    def __init__(self):
        super().__init__('ir_listener')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            IrIntensityVector, '/ir_intensity', self.ir_callback, qos_profile)
        self.get_logger().info("✅ Subscribed to /ir_intensity")

    def ir_callback(self, msg):
        global latest_ir_data
        latest_ir_data = [(r.header.frame_id, r.value) for r in msg.readings]


class HazardListener(Node):
    def __init__(self):
        super().__init__('hazard_listener')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile)
        self.get_logger().info("✅ Subscribed to /hazard_detection")

    def hazard_callback(self, msg):
        global latest_hazard_data
        if not msg.detections:
            latest_hazard_data = []
            return
        latest_hazard_data = [(d.header.frame_id, d.type) for d in msg.detections]


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
                    direction_history.append((timestamp, direction))
                    while direction_history and direction_history[0][0] < timestamp - 1.0:
                        direction_history.popleft()
                    if direction_history:
                        values = [v for t, v in direction_history]
                        avg_direction = sum(values) / len(values)
                        latest_direction = avg_direction
                    else:
                        latest_direction = None
            except Exception as e:
                print(f"⚠️ WebSocket error: {e}")
                await asyncio.sleep(1)


class DirectionController(Node):
    def __init__(self):
        super().__init__('direction_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

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


async def main():
    global OB_TRESHOLD, turning_speed

    # Initialiseer ROS nodes
    rclpy.init()
    ir_listener = IRListener()
    hazard_listener = HazardListener()
    battery_listener = BatteryListener()
    controller = DirectionController()

    # Open seriële verbinding
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print(f"✅ Seriële verbinding geopend op {SERIAL_PORT}")
    except Exception as e:
        print(f"❌ Kon seriële poort niet openen: {e}")
        sys.exit(1)

    # Start direction task
    direction_task = asyncio.create_task(get_direction())

    try:
        while rclpy.ok():
            rclpy.spin_once(ir_listener, timeout_sec=0.1)
            rclpy.spin_once(hazard_listener, timeout_sec=0.1)
            rclpy.spin_once(battery_listener, timeout_sec=0.1)

            if latest_battery_state is not None and latest_battery_state < MIN_BATTERY_LEVEL_PCT:
                print(f"🔋 Batterij bijna leeg ({latest_battery_state*100:.0f}%) - script stopt.")
                sys.exit(0)

            if latest_hazard_data:
                for frame, dtype in latest_hazard_data:
                    if frame in ["bump_left", "bump_front_left"]:
                        print(f"⚠️ Bump links gedetecteerd: {frame}")
                        controller.publish_manual_control(0.0, turning_speed)
                    if frame in ["bump_right", "bump_front_right"]:
                        print(f"⚠️ Bump rechts gedetecteerd: {frame}")
                        controller.publish_manual_control(0.0, -turning_speed)

            if latest_direction is not None:
                # Stuur servo
                angle_int = int(max(0, min(180, latest_direction)))
                ser.write(f"{angle_int}\n".encode())
                print(f"➡️ Servo angle gestuurd: {angle_int}")

                # Rij vooruit in AI mode
                controller.align_to_direction(forward_speed, latest_direction)

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")

    finally:
        ser.close()
        print("🔌 Seriële verbinding gesloten.")
        ir_listener.destroy_node()
        hazard_listener.destroy_node()
        battery_listener.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
