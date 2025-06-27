#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import IrIntensityVector  # Gebruik hier je echte msg type

class IrViewer(Node):
    def __init__(self):
        super().__init__('ir_viewer')
        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.listener_callback,
            10
        )
        self.get_logger().info("✅ Subscribed to /ir_intensity")

    def listener_callback(self, msg):
        print("==== /ir_intensity ontvangen ====")
        print(f"Header: {msg.header}")
        for reading in msg.readings:
            frame = reading.header.frame_id
            value = reading.value
            print(f"- {frame}: {value}")
        print("=================================")

def main(args=None):
    rclpy.init(args=args)
    node = IrViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
