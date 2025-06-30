#!/usr/bin/env python3

import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist
from rclpy.qos import  QoSProfile, QoSReliabilityPolicy
from rclpy.publisher import Publisher
import time
from collections import deque

SIGNALING_SERVER_DIRECTION = "ws://192.168.0.74:9000"
MAX_ANGULAR = 5  # Max angular speed in rad/s
OB_TRESHOLD = 100

latest_direction = None
latest_detected = None
latest_ir_data = None

# Een buffer met (timestamp, waarde)
direction_history = deque()


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
                direction = data.get("direction_angle")
                currentDetected = data.get("detected")
                timestamp = time.time()

                if (currentDetected is None):
                    last_no_detection = time.time()
                else:
                    last_no_detection = False
                    latest_detected = True
                
                if (last_no_detection > timestamp - 1.0):
                    latest_detected = False


                if direction is not None:
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
        print (f"➡️ x: {twist.linear.x:.2f}, z: {twist.angular.z:.2f}")
        self.publisher.publish(twist)

    def align_to_direction(self, linear_x, angle):
        error = angle - 90.0
        proportion = error / 90.0
        angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, proportion * MAX_ANGULAR))
        self.publish_manual_control(linear_x, angular_z)



async def main():
    global OB_TRESHOLD
    rclpy.init()
    ir_listener = IRListener()

    controller = DirectionController()

    direction_task = asyncio.create_task(get_direction())



    try:
        while rclpy.ok():
            rclpy.spin_once(ir_listener, timeout_sec=0.1)

            if latest_direction is not None:
                print(f"🎯 Direction angle: {latest_direction}, Detected: {latest_detected}")

            if latest_ir_data is not None:
                for frame, value in latest_ir_data:
                    if (value > 50) :
                        print(f"   - {frame}: {value}")

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
                AIMode = True

                if latest_detected:
                    AIMode = True
                    print (f"🎯 AI richting: {latest_direction}")
                    if latest_direction < 87:
                        print(f"🧭 AI richting: {latest_direction}")
                        decision = "AI_RIGHT"
                    elif latest_direction > 93:
                        print (f"🧭 AI richting: {latest_direction}")
                        decision = "AI_LEFT"
                    else:
                        print (f"🧭 AI richting: {latest_direction}")
                        decision = "AI_FORWARD"

                if not latest_detected:
                    AIMode = False
                    controller.publish_manual_control(0.0,0.5)

                if front_obstacle:
                    AIMode = False
                    if left_obstacle and right_obstacle:
                        decision = "OB_LEFT" # turn left when both sides are blocked
                    
                    if left_obstacle:
                        decision = "OB_RIGHT"
                    
                    if right_obstacle:
                        decision = "OB_LEFT"
            
                if printNeeded:
                    print(f"🧭 BESLISSING: {decision}")

                linear_x=0.1
                angular_z=0.0

                if (AIMode == True):
                    controller.align_to_direction(linear_x,latest_direction)


                '''
                if (AIMode == False):
                    if decision == "OB_LEFT":
                        linear_x = 0.0
                        angular_z = -0.5
                        controller.publish_manual_control(linear_x, angular_z)
                    elif decision == "OB_RIGHT":
                        linear_x = 0.0
                        angular_z = 0.5
                        controller.publish_manual_control(linear_x, angular_z)
                    else :
                        linear_x = 0.1
                        angular_z = 0.0
                        controller.publish_manual_control(linear_x, angular_z)
                '''
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("⏹️ Afgesloten door gebruiker")

    finally:
        ir_listener.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
