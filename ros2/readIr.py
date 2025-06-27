#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector

OBSTACLE_THRESHOLD = 20  # Grootte waarboven obstakel wordt gezien

class IrController(Node):
    def __init__(self):
        super().__init__('ir_controller')

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

        self.obstacle_detected = False
        self.get_logger().info("✅ Subscribed to /ir_intensity met BEST_EFFORT QoS")

    def listener_callback(self, msg):
        obstacle = False
        for reading in msg.readings:
            frame = reading.header.frame_id
            value = reading.value
            print(f"- {frame}: {value}")
            if value > OBSTACLE_THRESHOLD:
                obstacle = True

        twist = Twist()
        if obstacle:
            if not self.obstacle_detected:
                self.get_logger().info("🚧 Obstakel gedetecteerd: stoppen")
            self.obstacle_detected = True
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            if self.obstacle_detected:
                self.get_logger().info("✅ Geen obstakel meer: vooruit rijden")
            self.obstacle_detected = False
            twist.linear.x = 0.2  # snelheid vooruit
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = IrController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
