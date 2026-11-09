import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import os


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.prev_blurred = None

        # Каталог для збереження результатів
        self.save_dir = os.path.expanduser(
            '~/ros2_ws/results'
        )

        os.makedirs(self.save_dir, exist_ok=True)

        # Підписка на топік камери
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info(
            'Vision Node started. Waiting for /camera/image'
        )

    def image_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:
            self.get_logger().error(
                f'CvBridge error: {e}'
            )
            return

        # 1. Grayscale
        gray = cv2.cvtColor(
            cv_image,
            cv2.COLOR_BGR2GRAY
        )

        # 2. Gaussian Blur
        blurred = cv2.GaussianBlur(
            gray,
            (5, 5),
            0
        )

        # 3. Threshold
        _, thresh = cv2.threshold(
            blurred,
            127,
            255,
            cv2.THRESH_BINARY
        )

        # 4. Canny Edge Detection
        edges = cv2.Canny(
            blurred,
            50,
            150
        )

        # 5. Motion Detection
        motion_frame = cv_image.copy()

        if self.prev_blurred is not None:

            diff = cv2.absdiff(
                self.prev_blurred,
                blurred
            )

            _, motion_mask = cv2.threshold(
                diff,
                25,
                255,
                cv2.THRESH_BINARY
            )

            motion_mask = cv2.dilate(
                motion_mask,
                None,
                iterations=2
            )

            contours, _ = cv2.findContours(
                motion_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            motion_count = 0

            for contour in contours:

                if cv2.contourArea(contour) > 500:

                    x, y, w, h = cv2.boundingRect(
                        contour
                    )

                    cv2.rectangle(
                        motion_frame,
                        (x, y),
                        (x + w, y + h),
                        (0, 0, 255),
                        2
                    )

                    motion_count += 1

            if motion_count > 0:
                self.get_logger().info(
                    f'Motion detected: {motion_count} region(s)'
                )

        self.prev_blurred = blurred.copy()

        # Перетворення одноканальних зображень у BGR
        gray_bgr = cv2.cvtColor(
            gray,
            cv2.COLOR_GRAY2BGR
        )

        blurred_bgr = cv2.cvtColor(
            blurred,
            cv2.COLOR_GRAY2BGR
        )

        thresh_bgr = cv2.cvtColor(
            thresh,
            cv2.COLOR_GRAY2BGR
        )

        edges_bgr = cv2.cvtColor(
            edges,
            cv2.COLOR_GRAY2BGR
        )

        # Додавання підписів до зображень
        def add_label(img, text):

            canvas = img.copy()

            cv2.rectangle(
                canvas,
                (0, 0),
                (640, 35),
                (20, 20, 20),
                -1
            )

            cv2.putText(
                canvas,
                text,
                (15, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            return canvas

        img1 = add_label(cv_image, 'Original')
        img2 = add_label(gray_bgr, 'Grayscale')
        img3 = add_label(blurred_bgr, 'Gaussian Blur')
        img4 = add_label(thresh_bgr, 'Threshold')
        img5 = add_label(edges_bgr, 'Edges (Canny)')
        img6 = add_label(motion_frame, 'Motion')

        # Формування мозаїки 3x2
        row1 = np.hstack([
            cv2.resize(img1, (320, 240)),
            cv2.resize(img2, (320, 240)),
            cv2.resize(img3, (320, 240))
        ])

        row2 = np.hstack([
            cv2.resize(img4, (320, 240)),
            cv2.resize(img5, (320, 240)),
            cv2.resize(img6, (320, 240))
        ])

        mosaic = np.vstack([
            row1,
            row2
        ])

        # Відображення результату
        cv2.imshow('Vision', mosaic)

        key = cv2.waitKey(1) & 0xFF

        # Збереження результатів клавішею S
        if key == ord('s'):

            cv2.imwrite(
                os.path.join(self.save_dir, 'original.png'),
                cv_image
            )

            cv2.imwrite(
                os.path.join(self.save_dir, 'grayscale.png'),
                gray
            )

            cv2.imwrite(
                os.path.join(self.save_dir, 'gaussian_blur.png'),
                blurred
            )

            cv2.imwrite(
                os.path.join(self.save_dir, 'threshold.png'),
                thresh
            )

            cv2.imwrite(
                os.path.join(self.save_dir, 'canny_edges.png'),
                edges
            )

            cv2.imwrite(
                os.path.join(self.save_dir, 'motion.png'),
                motion_frame
            )

            cv2.imwrite(
                os.path.join(self.save_dir, 'mosaic.png'),
                mosaic
            )

            self.get_logger().info(
                f'Results saved to {self.save_dir}'
            )

        elif key == ord('q'):
            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
