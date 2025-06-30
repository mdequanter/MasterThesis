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
from rclpy.qos import  QoSProfile, QoSReliabilityPolicy
from rclpy.publisher import Publisher
import time
from collections import deque

SIGNALING_SERVER_DIRECTION = "ws://192.168.0.74:9000"
MAX_ANGULAR = 1 # Max angular speed in rad/s
OB_TRESHOLD = 50
MIN_BATTERY_LEVEL_PCT = 0.2  # Minimum battery level percentage to continue operation
forward_speed = 0.1  # Standaard vooruit snelheid
turning_speed = 0.5  # Standaard draai snelheid

latest_direction = None
latest_detected = None
latest_ir_data = None
latest_hazard_data = None
latest_battery_state = None


# Een buffer met (timestamp, waarde)
direction_history = deque()

class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            qos_profile
        )

        self.get_logger().info("✅ Subscribed to /battery_state")

    def battery_callback(self, msg):
        global latest_battery_state
        latest_battery_state = msg.percentage



# ROS Node om IR-values op te halen
class IRListener(Node):
    def __init__(self):
        super().__init__('ir_listener')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.ir_callback,
            qos_profile
        )

        self.get_logger().info("✅ Subscribed to /ir_intensity (IrIntensityVector)")

    def ir_callback(self, msg):
        global latest_ir_data
        latest_ir_data = [(r.header.frame_id, r.value) for r in msg.readings]

# ROS Node om Hazard detections op te halen
class HazardListener(Node):
    def __init__(self):
        super().__init__('hazard_listener')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.hazard_callback,
            qos_profile
        )

        self.get_logger().info("✅ Subscribed to hazard topic")

    def hazard_callback(self, msg):
            global latest_hazard_data
            if not msg.detections:
                #print("✅ Geen detecties")
                latest_hazard_data = []
                return

            latest_hazard_data = []
            #print("⚠️ Hazard detectie(s):")
            for detection in msg.detections:
                frame = detection.header.frame_id
                dtype = detection.type
                #print(f" - Frame: {frame}, Type: {dtype}")
                latest_hazard_data.append((frame, dtype))


# Asynchrone taak om direction op te halen
async def get_direction():
    global latest_direction, latest_detected
    last_no_detection = False
    print(f"📡 Verbinden met {SIGNALING_SERVER_DIRECTION}")
    async with websockets.connect(SIGNALING_SERVER_DIRECTION) as websocket:
        print("✅ Verbonden met direction server")
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                #print (f"📥 Ontvangen data: {data}")
                direction = data.get("direction_angle")
                timestamp = time.time()
                if direction is not None:
                    #print (f"📥 Ontvangen richting: {direction:.2f}°")
                    direction_history.append((timestamp, direction))
                    last_timestamp = direction_history[-1][0]

                    # Oude waarden (>1 sec) verwijderen
                    while direction_history and direction_history[0][0] < timestamp - 1.0:
                        direction_history.popleft()

                    # Gemiddelde berekenen
                    if direction_history:
                        values = [v for t, v in direction_history]
                        avg_direction = sum(values) / len(values)
                        latest_direction = avg_direction
                    else:
                        latest_direction = None
                # print(f"🎯 Gemiddelde richting laatste seconde: {latest_direction:.2f}")

            except Exception as e:
                print(f"⚠️ WebSocket error: {e}")
                await asyncio.sleep(1)

class DirectionController(Node):
    def __init__(self):
        super().__init__('direction_controller')
        self.publisher: Publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_publish_time = time.time()

    def publish_manual_control(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        #print (f"➡️ x: {twist.linear.x:.2f}, z: {twist.angular.z:.2f}")
        self.publisher.publish(twist)

    def align_to_direction(self, linear_x, angle):
        error = angle - 90.0
        proportion = error / 90.0
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, proportion * MAX_ANGULAR))
        self.publish_manual_control(linear_x, angular_z)



async def main():
    global OB_TRESHOLD, turning_speed
    rclpy.init()
    ir_listener = IRListener()
    hazard_listener = HazardListener()
    battery_listener = BatteryListener()


    controller = DirectionController()

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
                #print("⚠️ Er is een hazard! Details:")
                for frame, dtype in latest_hazard_data:
                        if (frame == "bump_left" or frame == "bump_front_left"):
                            print(f"⚠️ Bump links gedetecteerd: {frame}")
                            controller.publish_manual_control(0.0,turning_speed)
                        if (frame == "bump_right" or frame == "bump_front_right"):
                            print(f"⚠️ Bump rechts gedetecteerd: {frame}")
                            controller.publish_manual_control(0.0,-turning_speed)
                            

            # 🧠 Beslissingslogica
            decision = "Onbekend"
            if latest_ir_data:
                # Categoriseer sensoren
                front_sensors = ["ir_intensity_front_left", "ir_intensity_front_center_left",
                                 "ir_intensity_front_center_right", "ir_intensity_front_right"]
                left_sensors = ["ir_intensity_left", "ir_intensity_side_left"]
                right_sensors = ["ir_intensity_right"]

                # Detecteer obstakels
                front_obstacle = any(
                    value > OB_TRESHOLD for frame, value in latest_ir_data if frame in front_sensors
                )
                left_obstacle = any(
                    value > OB_TRESHOLD for frame, value in latest_ir_data if frame in left_sensors
                )
                right_obstacle = any(
                    value > OB_TRESHOLD for frame, value in latest_ir_data if frame in right_sensors
                )

                printNeeded = True
                # Bepaal actie
                AIMode = False

                if latest_direction is not None:
                    AIMode = True
                    controller.align_to_direction(forward_speed, latest_direction)
                    decision = "AIMODE"
                
                if front_obstacle:
                    AIMode = False
                    if (left_obstacle and right_obstacle):
                        if (left_obstacle > right_obstacle):
                            controller.publish_manual_control(0.0,-turning_speed)
                        else:
                            controller.publish_manual_control(0.0, turning_speed)
                    if left_obstacle:
                        controller.publish_manual_control(forward_speed,-turning_speed)
                    if right_obstacle:
                        controller.publish_manual_control(forward_speed,turning_speed)
                                    
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")

    finally:
        ir_listener.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
