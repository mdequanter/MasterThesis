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
        left_values = []
        right_values = []

        # Verzamel waarden per zijde
        for reading in msg.readings:
            frame = reading.header.frame_id
            value = reading.value
            print(f"- {frame}: {value}")

            if "front" in frame and value > OBSTACLE_THRESHOLD:
                obstacle_front = True
            elif "left" in frame:
                left_values.append(value)
            elif "right" in frame:
                right_values.append(value)

        twist = Twist()

        if obstacle_front:
            # Bepaal waar meer ruimte is (kleinere waarde = meer ruimte)
            avg_left = sum(left_values) / len(left_values) if left_values else OBSTACLE_THRESHOLD
            avg_right = sum(right_values) / len(right_values) if right_values else OBSTACLE_THRESHOLD
            self.get_logger().info(f"🚧 Obstakel VOORAAN: links gem={avg_left:.1f}, rechts gem={avg_right:.1f}")

            if avg_left < avg_right:
                self.get_logger().info("↩️ Meer ruimte LINKS: draaien naar links")
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            else:
                self.get_logger().info("↪️ Meer ruimte RECHTS: draaien naar rechts")
                twist.linear.x = 0.0
                twist.angular.z = -0.5

        elif any(v > OBSTACLE_THRESHOLD for v in left_values):
            self.get_logger().info("↪️ Obstakel LINKS: draaien naar rechts")
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        elif any(v > OBSTACLE_THRESHOLD for v in right_values):
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
