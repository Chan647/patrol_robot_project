import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
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

        self.sub_image = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)

    def send_start(self):
        msg = Bool()
        msg.data = True
        self.pub_start.publish(msg)

    def send_stop(self):
        msg = Bool()
        msg.data = True
        self.pub_stop.publish(msg)
    
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.gui.latest_frame = frame

class Patrolgui(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Patrol GUI')
        self.resize(700,600)

        self.camera_label = QLabel()
        self.camera_label.setFixedSize(640,480)
        self.camera_label.setAlignment(Qt.AlignCenter)

        self.btn_drive = QPushButton("DRIVING")
        self.btn_stop = QPushButton("STOP")
        self.btn_drive.setFixedHeight(70)
        self.btn_stop.setFixedHeight(70)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.btn_drive)
        btn_layout.addWidget(self.btn_stop)

        layout = QVBoxLayout()
        layout.addWidget(self.camera_label)
        layout.addLayout(btn_layout)
        self.setLayout(layout)

        self.latest_frame = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def btn_connect(self, ros):
        self.btn_drive.clicked.connect(ros.send_start)
        self.btn_stop.clicked.connect(ros.send_stop)

    def update_frame(self):
        if self.latest_frame is None:
            return

        rgb = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w

        image = QImage(
            rgb.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888
        )

        self.camera_label.setPixmap(QPixmap.fromImage(image))

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

