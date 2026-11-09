import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class CameraSimNode(Node):

    def __init__(self):
        super().__init__('camera_sim_node')

        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image',
            10
        )

        self.timer = self.create_timer(
            1.0 / 15.0,
            self.timer_callback
        )

        self.bridge = CvBridge()

        self.circle_x = 100
        self.dx = 4

        self.get_logger().info(
            'Camera simulator started: 640x480, 15 FPS'
        )

    def timer_callback(self):

        frame = np.full(
            (480, 640, 3),
            40,
            dtype=np.uint8
        )

        # Помаранчевий прямокутник
        cv2.rectangle(
            frame,
            (80, 180),
            (180, 280),
            (30, 105, 210),
            -1
        )

        # Білий квадрат
        cv2.rectangle(
            frame,
            (260, 180),
            (360, 280),
            (230, 230, 230),
            -1
        )

        # Жовтий трикутник
        pts = np.array(
            [[500, 180], [450, 280], [550, 280]],
            np.int32
        )

        cv2.fillPoly(
            frame,
            [pts],
            (0, 215, 255)
        )

        # Рухоме коло
        self.circle_x += self.dx

        if self.circle_x > 580 or self.circle_x < 60:
            self.dx = -self.dx

        cv2.circle(
            frame,
            (self.circle_x, 380),
            30,
            (0, 255, 100),
            -1
        )

        # Текст на зображенні
        cv2.putText(
            frame,
            'SIM CAMERA',
            (20, 460),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 180, 180),
            1
        )

        # Конвертація OpenCV -> ROS Image
        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = CameraSimNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
