import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
import math

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class NavigationSegmenter(Node):
    def __init__(self):
        super().__init__('navigation_segmenter')

        self.model = YOLO('ros2/models/botopiasim.pt', verbose=False)
        self.frame_size = (640, 360)
        self.view_angle_deg = 90  # Enkel visualisatie
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("Received empty frame.")
            return

        frame = cv2.resize(frame, self.frame_size)
        center_x = self.frame_size[0] // 2
        base_y = self.frame_size[1] - 1

        # Teken zichtkegel van 20 graden
        half_angle_rad = math.radians(self.view_angle_deg / 2)
        cone_length = 150  # pixels omhoog
        top_y = base_y - cone_length
        left_x = int(center_x - cone_length * math.tan(half_angle_rad))
        right_x = int(center_x + cone_length * math.tan(half_angle_rad))

        cone_pts = np.array([[center_x, base_y], [left_x, top_y], [right_x, top_y]], np.int32)
        cv2.polylines(frame, [cone_pts], isClosed=True, color=(0, 255, 255), thickness=2)

        results = self.model(frame, conf=0.5, verbose=False)
        steering_direction = 0

        overlay = frame.copy()

        for result in results:
            if result.masks is not None and result.boxes is not None:
                for i, mask_xy in enumerate(result.masks.xy):
                    class_id = int(result.boxes.cls[i].item())
                    class_name = self.model.names[class_id]

                    if class_name == 'street':
                        points = np.array(mask_xy, dtype=np.int32)

                        # Vul het mask op het overlaybeeld met een grijze transparante kleur
                        cv2.fillPoly(overlay, [points], color=(150, 150, 150))  # grijze overlay

                        # Bereken zwaartepunt
                        M = cv2.moments(points)
                        if M['m00'] != 0:
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])
                            steering_direction = cx - center_x
                            cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                        break  # enkel eerste street segment
        # Voeg overlay transparant toe
        alpha = 0.5  # mate van transparantie
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)


        # Pijl onderaan het beeld
        arrow_origin = (center_x, base_y - 10)
        arrow_length = 60
        arrow_tip = (center_x + int(np.clip(steering_direction * 0.5, -arrow_length, arrow_length)),
                     base_y - 60)

        # Parameters
        self.angular_speed = 0.0

        twist = Twist()

        if (steering_direction < -10):
            self.angular_speed = 0.1
        elif (steering_direction > 10):
            self.angular_speed = -0.1
        else:
            self.angular_speed = 0.0   
        
        if (self.angular_speed != 0.0):
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Steering command: angular={self.angular_speed}")

        cv2.arrowedLine(frame, arrow_origin, arrow_tip, (255, 0, 0), 4, tipLength=0.5)
        cv2.putText(frame, "Steering: {}".format(
            "LEFT" if steering_direction < -10 else "RIGHT" if steering_direction > 10 else "STRAIGHT"),
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Direction Guidance", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("User exit requested.")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            exit()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
