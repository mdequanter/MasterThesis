#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector

OBSTACLE_THRESHOLD = 100  # Grootte waarboven obstakel wordt gezien

class IrAvoidanceController(Node):
    def __init__(self):
        super().__init__('ir_avoidance_controller')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.listener_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("✅ Subscribed to /ir_intensity met BEST_EFFORT QoS")

    def listener_callback(self, msg):
        obstacle_front = False
        obstacle_left = False
        obstacle_right = False

        # Bekijk alle metingen
        for reading in msg.readings:
            frame = reading.header.frame_id
            value = reading.value
            print(f"- {frame}: {value}")

            if value > OBSTACLE_THRESHOLD:
                if "front" in frame:
                    obstacle_front = True
                elif "left" in frame:
                    obstacle_left = True
                elif "right" in frame:
                    obstacle_right = True

        twist = Twist()

        # Beslis op basis van detectie
        if obstacle_front:
            self.get_logger().info("🚧 Obstakel VOORAAN: stoppen")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif obstacle_left:
            self.get_logger().info("↪️ Obstakel LINKS: draaien naar rechts")
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        elif obstacle_right:
            self.get_logger().info("↩️ Obstakel RECHTS: draaien naar links")
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        else:
            twist.linear.x = 0.2  # vooruit
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = IrAvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
