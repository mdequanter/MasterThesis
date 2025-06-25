# turtlebot3Drive.py
# Before running this script, ensure you have the necessary ROS 2 packages installed and sourced.
# Make sure Gazebo is running with the TurtleBot3 model loaded and correct ROS_DOMAIN_ID  is set
# install the required packages as in requirements.txt

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from std_srvs.srv import Empty

class Command:
    def __init__(self, action, value=None):
        self.action = action
        self.value = value

class TurtleBot3Commander(Node):
    def __init__(self):
        super().__init__('turtlebot3_commander')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.pose = None
        self.yaw = None
        self.initial_pose = None
        self.initial_yaw = None

        # Parameters
        self.linear_speed = 0.1
        self.angular_speed = 0.3

        # Command list
        self.commands = [
            Command("SPEED", 0.1),
            Command("FORWARD", 1.85),
            Command("CCWTURN", 90),
            Command("FORWARD", 1.3),
            Command("CCWTURN", 90),
            Command("FORWARD", 1.95),
            Command("CCWTURN", 90),
            Command("FORWARD", 1.3),
            Command("CCWTURN", 90),
            Command("STOP")
        ]
        self.current_cmd_index = 0
        self.state = 'IDLE'

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def control_loop(self):
        if self.pose is None or self.yaw is None:
            return

        if self.current_cmd_index >= len(self.commands):
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Alle commando's uitgevoerd.")
            rclpy.shutdown()
            return

        cmd = self.commands[self.current_cmd_index]
        twist = Twist()

        if self.state == 'IDLE':
            if cmd.action == "SPEED":
                self.linear_speed = float(cmd.value)
                self.angular_speed = float(cmd.value * 2)
                self.get_logger().info(f"Snelheid aangepast naar {self.linear_speed} m/s")
                self.current_cmd_index += 1

            elif cmd.action == "FORWARD":
                self.initial_pose = self.pose
                self.state = 'MOVING'

            elif cmd.action in ["CCWTURN", "CWTURN"]:
                self.initial_yaw = self.yaw
                self.state = 'TURNING'

            elif cmd.action == "STOP":
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("STOP ontvangen. Robot wordt afgesloten.")
                self.destroy_node()
                rclpy.shutdown()
                exit()
                return

        elif self.state == 'MOVING':
            dx = self.pose.x - self.initial_pose.x
            dy = self.pose.y - self.initial_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < cmd.value:
                twist.linear.x = self.linear_speed
            else:
                self.get_logger().info(f"{cmd.value} meter gereden.")
                self.state = 'IDLE'
                self.current_cmd_index += 1

        elif self.state == 'TURNING':
            delta = self.normalize_angle(self.yaw - self.initial_yaw)
            target_rad = math.radians(cmd.value)
            direction = 1 if cmd.action == "CCWTURN" else -1

            if abs(delta) < abs(target_rad):
                twist.angular.z = direction * self.angular_speed
            else:
                self.get_logger().info(f"{cmd.value}° gedraaid.")
                self.state = 'IDLE'
                self.current_cmd_index += 1

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Commander()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
