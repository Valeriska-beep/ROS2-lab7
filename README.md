# Лабораторна робота №7

## Обробка відеопотоку та базовий комп'ютерний зір (OpenCV + ROS 2)

### Підготовка середовища та встановлення залежностей

Відкриваємо термінал Ubuntu. Оновлюємо список пакетів, встановлюємо необхідні бібліотеки для роботи з відеопотоком та OpenCV.

```bash
sudo apt update
```

```bash
sudo apt install -y ros-humble-cv-bridge python3-opencv python3-numpy ros-humble-rqt-image-view tree
```

Активуємо ROS 2 Humble та робочий простір:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Перевіряємо встановлені бібліотеки:

```bash
python3 -c "import cv2, numpy; from cv_bridge import CvBridge; print('OpenCV:', cv2.__version__); print('NumPy:', numpy.__version__); print('cv_bridge: OK')"
```

### Створення ROS 2 пакета

Створюємо окремий пакет `vision_pkg`, не змінюючи попередні лабораторні роботи (`motion_control`, `robot_gui`, `sensor_node_pkg` та інші).

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name vision_node vision_pkg --dependencies rclpy sensor_msgs cv_bridge
```

Перевіряємо структуру пакета:

```bash
tree ~/ros2_ws/src/vision_pkg
```

Переходимо до Python-модуля та створюємо два файли:

```bash
cd ~/ros2_ws/src/vision_pkg/vision_pkg
touch camera_sim_node.py
touch vision_node.py
```

Робимо файли виконуваними:

```bash
chmod +x camera_sim_node.py vision_node.py
```

---

### Створення ноди симулятора камери (camera_sim_node.py)

Ця нода імітує роботу камери. Вона генерує зображення розміром 640×480 пікселів із трьома нерухомими геометричними фігурами та одним рухомим колом.

Згенеровані кадри публікуються в топік `/camera/image` з частотою приблизно 15 кадрів за секунду (15 FPS).

Відкриваємо файл:

```bash
nano ~/ros2_ws/src/vision_pkg/vision_pkg/camera_sim_node.py
```

2. Вставляємо повний код:

```python
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
```

### Створення Vision Node (vision_node.py)

Нода отримує зображення з топіка `/camera/image`, конвертує повідомлення ROS 2 у формат OpenCV за допомогою `cv_bridge` та виконує такі операції:

1. Grayscale — перетворення зображення у відтінки сірого.
2. Gaussian Blur — згладжування зображення.
3. Threshold — порогова бінаризація.
4. Canny Edge Detection — визначення меж об'єктів.
5. Motion Detection — виявлення рухомих об'єктів.

Результати відображаються в одному вікні у вигляді мозаїки 3×2.

Відкриваємо файл:

```bash
nano ~/ros2_ws/src/vision_pkg/vision_pkg/vision_node.py
```

```python
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
```

### Налаштування setup.py

Тепер реєструємо обидві ноди, щоб ROS 2 міг запускати їх через команду `ros2 run`.

Відкриваємо файл конфігурації:

```bash
nano ~/ros2_ws/src/vision_pkg/setup.py
```

Замінюємо вміст файлу таким кодом:

```python
from setuptools import setup, find_packages

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='ROS 2 OpenCV Vision Processing Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_pkg.vision_node:main',
            'camera_sim_node = vision_pkg.camera_sim_node:main',
        ],
    },
)
```

### Перевірка синтаксису та збірка пакета

Перед збіркою перевіряємо Python-файли на наявність синтаксичних помилок.
Перевіряємо код симулятора:

```bash
python3 -m py_compile ~/ros2_ws/src/vision_pkg/vision_pkg/camera_sim_node.py
```

Перевіряємо код Vision Node:

```bash
python3 -m py_compile ~/ros2_ws/src/vision_pkg/vision_pkg/vision_node.py
```

Якщо команди не виводять помилок, синтаксис файлів правильний.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

Збираємо новий пакет:

```bash
colcon build --packages-select vision_pkg
source ~/ros2_ws/install/setup.bash
```

Перевіряємо зареєстровані ноди:

```bash
ros2 pkg executables vision_pkg
```

Перевіряємо структуру пакета:

```bash
tree ~/ros2_ws/src/vision_pkg
```

### Контрольна точка №1

На цьому етапі пакет `vision_pkg` повинен успішно зібратися, а ROS 2 має бачити дві виконувані ноди:

- `camera_sim_node` — генератор відеопотоку.
- `vision_node` — обробник зображень.

---

### Запуск і тестування системи

Для перевірки роботи використовуємо три окремі термінали Ubuntu.

Термінал №1 — запуск симулятора камери

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run vision_pkg camera_sim_node
```

Нода починає генерувати кадри та публікувати їх у `/camera/image`.

Термінал №2 — перевірка відеопотоку


```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

Перевіряємо частоту передачі кадрів:

```bash
ros2 topic hz /camera/image
```

Очікуване значення — приблизно 15 Hz.

Зупиняємо перевірку через `Ctrl + C`.

Перевіряємо ширину кадру:

```bash
ros2 topic echo /camera/image --once --field width
```

Перевіряємо висоту кадру:

```bash
ros2 topic echo /camera/image --once --field height
```

Термінал №3 — запуск Vision Node

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Запускаємо ноду комп'ютерного зору:

```bash
ros2 run vision_pkg vision_node
```

Має відкритися вікно `Vision` із шістьма панелями:

1. Original — початкове зображення.
2. Grayscale — зображення у відтінках сірого.
3. Gaussian Blur — розмите зображення.
4. Threshold — бінаризоване зображення.
5. Edges (Canny) — виявлені межі об'єктів.
6. Motion — результат виявлення руху.

У шостій панелі рухомий об'єкт повинен виділятися прямокутником.

---

### Додаткова перевірка через rqt_image_view

Для окремої демонстрації відеопотоку можна використати графічний інструмент `rqt_image_view`.

Відкриваємо четвертий термінал:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Запускаємо інструмент:

```bash
ros2 run rqt_image_view rqt_image_view
```

Має відобразитися зображення, яке генерує симулятор камери.

Це додатково підтверджує, що відеопотік передається через ROS 2.

### Збереження результатів обробки

У програмі передбачено збереження результатів за допомогою клавіші `S`.
Клацаємо мишкою по вікну `Vision` та натискаємо `S`. Після цього результати автоматично зберігаються у каталозі:

```bash
~/ros2_ws/results/
```

Перевіряємо наявність збережених файлів:

```bash
ls -lh ~/ros2_ws/results/
```

 За потреби відкриваємо каталог із результатами:**

```bash
xdg-open ~/ros2_ws/results/
```

