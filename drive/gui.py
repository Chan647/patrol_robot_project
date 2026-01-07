import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel,
    QPushButton, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

class GuiNode(Node):
    def __init__(self, gui):
        super().__init__('patrol_gui_node')
        self.gui = gui
        self.pub_start = self.create_publisher(Bool, '/patrol_start', 10)
        self.pub_stop = self.create_publisher(Bool, '/patrol_stop', 10)
        self.pub_auto = self.create_publisher(Bool, '/auto_mode', 10)
        self.pub_manual = self.create_publisher(Bool, '/manual_mode', 10)

        self.sub_image = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_path = self.create_subscription(Path, '/planned_path', self.path_callback,10)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

    def send_start(self):
        msg = Bool()
        msg.data = True
        self.pub_start.publish(msg)

    def send_stop(self):
        msg = Bool()
        msg.data = True
        self.pub_stop.publish(msg)

    def auto(self):
        msg = Bool()
        msg.data = True
        self.pub_auto.publish(msg)

    def manual(self):
        msg = Bool()
        msg.data = True
        self.pub_manual.publish(msg)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.gui.camera_frame = frame

    def map_callback(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h,w))

        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[data == 0]   = (245, 245, 245)
        img[data == 100] = (30, 30, 30)
        img[data < 0]    = (180, 180, 180)

        img = cv2.flip(img, 1)

        self.gui.map_resolution = msg.info.resolution
        self.gui.map_origin =(msg.info.origin.position.x, msg.info.origin.position.y)
        self.gui.map_image = img

    def path_callback(self, msg):
        paths = []
        for p in msg.poses:
            paths.append((p.pose.position.x, p.pose.position.y))
        self.gui.path_points = paths

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        self.gui.robot_pose = (p.x, p.y)


class Patrolgui(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Patrol GUI')
        self.resize(1280, 700)

        self.camera_label = QLabel()
        self.camera_label.setFixedSize(640,480)
        self.camera_label.setAlignment(Qt.AlignCenter)

        self.map_label = QLabel()
        self.map_label.setFixedSize(640,480)
        self.map_label.setAlignment(Qt.AlignCenter)

        self.btn_drive = QPushButton("DRIVING")
        self.btn_stop = QPushButton("STOP")
        self.btn_auto = QPushButton("AUTO")
        self.btn_manual = QPushButton("MANUAL")
        self.btn_drive.setFixedHeight(70)
        self.btn_stop.setFixedHeight(70)
        self.btn_auto.setFixedHeight(70)
        self.btn_manual.setFixedHeight(70)
        self.btn_drive.setStyleSheet("""QPushButton {background-color: #2ecc71; color: white; font-size: 20px; font-weight: bold; border: 2px solid #1e8449; border-radius: 8px;}
            QPushButton:hover {background-color: #27ae60;}""")

        self.btn_stop.setStyleSheet("""QPushButton {background-color: #e74c3c; color: white; font-size: 20px; font-weight:bold; border: 2px solid #922b21; border-radius: 8px;}
            QPushButton:hover {background-color: #c0392b;}""")

        self.btn_auto.setStyleSheet("""QPushButton {background-color: #3498db;color: white;font-size: 18px;font-weight: bold; border: 2px solid #1b4f72; border-radius: 8px;}
            QPushButton:hover {background-color: #2980b9;}""")

        self.btn_manual.setStyleSheet("""QPushButton {background-color: #f39c12; color: white; font-size: 18px; font-weight: bold; border: 2px solid #9c640c; border-radius: 8px;}
            QPushButton:hover {background-color: #e67e22;}""")

        top_layout = QHBoxLayout()
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(10)
        top_layout.addWidget(self.camera_label, stretch=1)
        top_layout.addWidget(self.map_label, stretch=1)

        btn_layout = QHBoxLayout()
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(10)
        btn_layout.addWidget(self.btn_drive, stretch=1)
        btn_layout.addWidget(self.btn_stop, stretch=1)

        btn2_layout = QHBoxLayout()
        btn2_layout.setContentsMargins(0, 0, 0, 0)
        btn2_layout.setSpacing(10)
        btn2_layout.addWidget(self.btn_auto, stretch=1)
        btn2_layout.addWidget(self.btn_manual, stretch=1)

        layout = QVBoxLayout()
        layout.addLayout(top_layout)
        layout.addLayout(btn_layout)
        layout.addLayout(btn2_layout)
        layout.setAlignment(Qt.AlignHCenter)
        self.setLayout(layout)

        self.camera_frame = None
        self.map_image = None
        self.map_resolution = None
        self.map_origin = None
        self.path_points = []
        self.robot_pose = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def btn_connect(self, ros):
        self.btn_drive.clicked.connect(ros.send_start)
        self.btn_stop.clicked.connect(ros.send_stop)
        self.btn_auto.clicked.connect(ros.auto)
        self.btn_manual.clicked.connect(ros.manual)

    def world_to_map(self, x, y):
        if self.map_origin is None or self.map_resolution is None:
            return None

        mx = (x - self.map_origin[0]) / self.map_resolution
        my = (y - self.map_origin[1]) / self.map_resolution
        mx = int(mx)
        my = int(my)

        my = self.map_image.shape[0] - my - 1

        if mx < 0 or my < 0 or mx >= self.map_image.shape[1] or my >= self.map_image.shape[0]:
            return None

        return mx, my

    def update_frame(self):
        if self.camera_frame is not None:
            cam = cv2.resize(self.camera_frame, (640, 480))
            cv2.rectangle(cam, (0, 0), (639, 479), (0, 0, 0), 3)

            cam_rgb = cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)
            cam_img = QImage(
                cam_rgb.data,
                640, 480,
                3 * 640,
                QImage.Format_RGB888
            )
            self.camera_label.setPixmap(QPixmap.fromImage(cam_img))

        if self.map_image is not None:
            m = self.map_image.copy()

            if self.path_points:
                pts = []
                for x, y in self.path_points:
                    p = self.world_to_map(x, y)
                    if p:
                        pts.append(p)
                if len(pts) > 1:
                    cv2.polylines(
                        m,
                        [np.array(pts, dtype=np.int32)],
                        False,
                        (0, 255, 0),
                        2
                    )

            if self.robot_pose:
                p = self.world_to_map(*self.robot_pose)
                if p:
                    cv2.circle(m, p, 6, (0, 0, 255), -1)

            m = cv2.resize(m, (640, 480))
            cv2.rectangle(m, (0, 0), (639, 479), (0, 0, 0), 3)

            map_rgb = cv2.cvtColor(m, cv2.COLOR_BGR2RGB)
            map_img = QImage(
                map_rgb.data,
                640, 480,
                3 * 640,
                QImage.Format_RGB888
            )
            self.map_label.setPixmap(QPixmap.fromImage(map_img))


def main():
    rclpy.init()

    app = QApplication(sys.argv)

    gui = Patrolgui()
    ros = GuiNode(gui)

    gui.btn_connect(ros)
    gui.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros, timeout_sec=0))
    ros_timer.start(10)

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()


