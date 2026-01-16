import sys
import cv2
import numpy as np
import rclpy
import datetime
import boto3
import uuid
import pymysql
import webbrowser
import threading
import math

from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem, QPushButton
from botocore.exceptions import ClientError
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from patrol_msgs.msg import Event

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel,
    QPushButton, QVBoxLayout, QHBoxLayout, QFrame, QSizePolicy, QHeaderView
)
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5.QtCore import QTimer, Qt


# 이미지 업로드 클라우도 MinIO 클라이언트 설정
# 이벤트 이미지 저장용
s3 = boto3.client(
    's3',
    endpoint_url='http://192.168.0.2:9000',
    aws_access_key_id='minioadmin',
    aws_secret_access_key='minioadmin',
    region_name='us-east-1'
)

BUCKET_NAME = 'patrolimage'

# 이벤트 발생 시 카메라 이미지를 MinIO에 업로드
# 업로드된 이미지의 URL 반환
def upload_image_to_minio(frame):
    filename = f"{uuid.uuid4()}.png"

    success, buffer = cv2.imencode('.png', frame)
    if not success:
        return None

    try:
        try:
            s3.head_bucket(Bucket=BUCKET_NAME)
        except ClientError:
            s3.create_bucket(Bucket=BUCKET_NAME)

        s3.put_object(
            Bucket=BUCKET_NAME,
            Key=filename,
            Body=buffer.tobytes(),
            ContentType='image/png'
        )

        return f"http://192.168.0.2:9000/{BUCKET_NAME}/{filename}"

    except Exception as e:
        print(f"[MinIO Upload Error] {e}")
        return None
    
# 이벤트 로그를 MySQL DB에 저장
def insert_event_log(
    situation,
    status,
    x,
    y,
    image_url
):
    conn = pymysql.connect(
        host='localhost',
        user='robot',
        password='robot1234',
        database='robot_db',
        charset='utf8mb4'
    )

    try:
        with conn.cursor() as cursor:
            sql = """
            INSERT INTO situation_log
            (situation, status, x, y, image_path, created_at)
            VALUES (%s, %s, %s, %s, %s, NOW())
            """
            cursor.execute(
                sql,
                (situation, status, x, y, image_url)
            )
        conn.commit()
    except Exception as e:
        print(f"[DB INSERT ERROR] {e}")
    finally:
        conn.close()

# 최신 이벤트 로그 조회
def fetch_event_logs(limit=100):
    conn = pymysql.connect(
        host='localhost',
        user='robot',
        password='robot1234',
        database='robot_db',
        charset='utf8mb4',
        cursorclass=pymysql.cursors.DictCursor
    )

    try:
        with conn.cursor() as cursor:
            sql = """
            SELECT
                situation,
                status,
                x,
                y,
                image_path,
                created_at
            FROM situation_log
            ORDER BY created_at DESC
            LIMIT %s
            """
            cursor.execute(sql, (limit,))
            return cursor.fetchall()

    except Exception as e:
        print(f"[DB FETCH ERROR] {e}")
        return []

    finally:
        conn.close()

# ROS 와 GUI 간 연결용 노드
# 센서 값, 이벤트 수신 및 GUI 버튼 제어
class GuiNode(Node):
    def __init__(self, gui):
        super().__init__('patrol_gui_node')
        self.gui = gui
        self.human_active = False

        self.pub_start = self.create_publisher(Bool, '/patrol_start', 10)
        self.pub_stop = self.create_publisher(Bool, '/patrol_stop', 10)
        self.pub_auto = self.create_publisher(Bool, '/auto_mode', 10)
        self.pub_manual = self.create_publisher(Bool, '/manual_mode', 10)
        self.pub_emergstop = self.create_publisher(Bool,'/emergency_stop', 10)
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.sub_image = self.create_subscription(CompressedImage, '/image_debug', self.image_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_path = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_event = self.create_subscription(Event, '/patrol_event', self.event_callback, 10)
        
    # GUI 버튼 클릭 시 로봇으로 명령 발송 
    def send_start(self):
        msg = Bool()
        msg.data = True
        self.pub_start.publish(msg)

    def send_stop(self):
        msg = Bool()
        msg.data = True
        self.pub_stop.publish(msg)

    def send_emergstop(self):
        msg = Bool()
        msg.data = True
        self.pub_emergstop.publish(msg)

    # AUTO 버튼 클릭 시 초기 위치 설정 후 자동 모드 전환
    def auto(self):
        self.pub_initial_pose()
        self._auto_timer = self.create_timer(5, self.auto_timer)

    def manual(self):
        msg = Bool()
        msg.data = True
        self.pub_manual.publish(msg)

    def auto_timer(self):
        msg = Bool()
        msg.data = True
        self.pub_auto.publish(msg)
        self._auto_timer.cancel()

    # 카메라 영상 토픽을 수신하여 GUI에 전달
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.gui.camera_frame = frame

    # 지도 토픽을 수신하여 GUI용 맵 이미지로 변환
    def map_callback(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        data = np.flipud(data)
        data = np.rot90(data, k=1)

        img = np.zeros((data.shape[0], data.shape[1], 3), dtype=np.uint8)
        img[data == 0]   = (245, 245, 245)
        img[data == 100] = (30, 30, 30)
        img[data < 0]    = (180, 180, 180)

        self.gui.map_resolution = msg.info.resolution
        self.gui.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        self.gui.grid_w = w
        self.gui.grid_h = h

        self.gui.map_image = img

    # 경로 토픽을 수신하여 주행 경로 저장
    def path_callback(self, msg):
        paths = []
        for p in msg.poses:
            paths.append((p.pose.position.x, p.pose.position.y))
        self.gui.path_points = paths
        
    # 로봇 위치 토픽을 수신하여 현재 위치 갱신
    def pose_callback(self, msg):
        p = msg.pose.pose.position
        self.gui.robot_pose = (p.x, p.y)

    # 특수 이벤트를 수신하여 이미지 캡처 및 UI 갱신(비상 정지, 사람 인식)
    def event_callback(self, msg):
        if msg.situation not in ["HUMAN DETECT", "EMERGENCY STOP"]:
            return

        if msg.situation == "HUMAN DETECT":
            if self.human_active:
                return
            self.human_active = True

            QTimer.singleShot(150, lambda: self.capture_event_image(msg))
            QTimer.singleShot(1000, lambda: setattr(self, "human_active", False))

        elif msg.situation == "EMERGENCY STOP":
            QTimer.singleShot(
                0,
                lambda: self.capture_event_image(msg)
            )

        self.update_event_UI(msg)

    # 이벤트 발생 시점의 카메라 영상 캡처
    # GUI에 표시 및 저장 준비
    def capture_event_image(self, msg):
        if self.gui.camera_frame is None:
            return

        frame = self.gui.camera_frame.copy()

        img = cv2.resize(frame, (280, 180))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        qimg = QImage(
            img_rgb.data,
            img_rgb.shape[1],
            img_rgb.shape[0],
            img_rgb.strides[0],
            QImage.Format_RGB888
        )
        self.gui.event_img.setPixmap(QPixmap.fromImage(qimg))

        threading.Thread(
            target=self.save_event,
            args=(msg, frame),
            daemon=True
        ).start()

    # 수신된 이벤트 정보를 GUI 화면에 표시
    def update_event_UI(self, msg):
        now = datetime.datetime.now()
        time_str = now.strftime("%H:%M:%S")

        self.gui.event_type.setText(f'<span style="color:green; font-weight:bold; font-size:25px">상황 : </span>' f'<span style="color:black; font-size:25px">{msg.situation}</span>')
        self.gui.event_msg.setText(f'<span style="color:green; font-weight:bold; font-size:25px">상태 : </span>' f'<span style="color:black; font-size:25px">{msg.status}</span>')
        self.gui.robot_x.setText(f'<span style="color:green; font-weight:bold; font-size:25px">X 좌표 : </span>' f'<span style="color:black; font-size:25px">{msg.x:.2f}</span>')
        self.gui.robot_y.setText(f'<span style="color:green; font-weight:bold; font-size:25px">y 좌표 : </span>' f'<span style="color:black; font-size:25px">{msg.y:.2f}</span>')
        self.gui.timestamp.setText(f'<span style="color:green; font-weight:bold; font-size:25px">현재 시간 : </span>' f'<span style="color:black; font-size:25px">{time_str}</span>')

    # 이벤트 이미지 업로드와 데이터베이스 저장을 별도 스레드에서 비동기로 처리
    def save_event(self, msg, frame):
        try:
            img_url = upload_image_to_minio(frame)
            if img_url:
                insert_event_log(
                    situation=msg.situation,
                    status=msg.status,
                    x=msg.x,
                    y=msg.y,
                    image_url=img_url
                )
        except Exception as e:
            print("[EVENT SAVE ERROR]", e)

    # 로봇의 초기 위치를 설정하여 AMCL 초기화용 토픽으로 발행
    def pub_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = -0.008
        msg.pose.pose.position.y = 0.029

        yaw = -0.041
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        self.pub_initialpose.publish(msg)

# GUI 버튼 및 레이아웃 설정
class Patrolgui(QWidget):
    def __init__(self):
        super().__init__()

        self.grid_w = None
        self.grid_h = None

        self.setWindowTitle('Patrol GUI')
        self.resize(1280, 700)

        self.camera_label = QLabel()
        self.camera_label.setFixedSize(600,480)
        self.camera_label.setAlignment(Qt.AlignCenter)

        self.map_label = QLabel()
        self.map_label.setFixedSize(600,480)
        self.map_label.setAlignment(Qt.AlignCenter)

        self.btn_drive = QPushButton("DRIVE")
        self.btn_stop = QPushButton("STOP")
        self.btn_auto = QPushButton("AUTO")
        self.btn_manual = QPushButton("MANUAL")
        self.btn_emerge = QPushButton("EMERGENCY STOP")

        self.btn_drive.setFixedHeight(70)
        self.btn_stop.setFixedHeight(70)
        self.btn_emerge.setFixedHeight(70)
        self.btn_auto.setFixedHeight(70)
        self.btn_manual.setFixedHeight(70)
        self.btn_drive.setStyleSheet("""QPushButton {background-color: #2ecc71; color: white; font-size: 20px; font-weight: bold; border: 2px solid #1e8449; border-radius: 8px;}
            QPushButton:hover {background-color: #58d68d;} QPushButton:pressed {background-color: #1e8449;}""")

        self.btn_stop.setStyleSheet("""QPushButton {background-color: #d35400; color: white; font-size: 20px; font-weight:bold; border: 2px solid #922b21; border-radius: 8px;}
            QPushButton:hover {background-color: #e67e22;} QPushButton:pressed {background-color: #a04000;}""")

        self.btn_auto.setStyleSheet("""QPushButton {background-color: #3498db;color: white;font-size: 18px;font-weight: bold; border: 2px solid #1b4f72; border-radius: 8px;}
            QPushButton:hover {background-color: #5dade2;} QPushButton:pressed {background-color: #1b4f72;}""")

        self.btn_manual.setStyleSheet("""QPushButton {background-color: #f5b041; color: white; font-size: 18px; font-weight: bold; border: 2px solid #9c640c; border-radius: 8px;}
            QPushButton:hover {background-color: #f8c471;} QPushButton:pressed {background-color: #b9770e;}""")

        self.btn_emerge.setStyleSheet("""QPushButton {background-color: #ff3b30; color: white; font-size: 18px; font-weight: bold; border: 2px solid #5c0000; border-radius: 8px;}
            QPushButton:hover {background-color: #ff5a52;} QPushButton:pressed {background-color: #c1271c;}""")

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

        left_layout = QVBoxLayout()
        left_layout.addLayout(top_layout)
        left_layout.addLayout(btn_layout)
        left_layout.addLayout(btn2_layout)
        left_layout.addWidget(self.btn_emerge)
        left_layout.setAlignment(Qt.AlignHCenter)

        self.event_type = QLabel('<span style="color:green; font-weight:bold; font-size:25px">상황 :</span>')
        self.event_msg  = QLabel('<span style="color:green; font-weight:bold; font-size:25px">상태 :</span>')
        self.robot_x    = QLabel('<span style="color:green; font-weight:bold; font-size:25px">X 좌표 :</span>')
        self.robot_y    = QLabel('<span style="color:green; font-weight:bold; font-size:25px">Y 좌표 :</span>')
        self.timestamp  = QLabel('<span style="color:green; font-weight:bold; font-size:25px">현재 시간 :</span>')

        self.event_frame = QFrame()
        self.event_frame.setFrameShape(QFrame.Box)
        self.event_frame.setLineWidth(3)
        self.event_frame.setFixedWidth(600)
        self.event_frame.setStyleSheet("""QFrame {background-color: #f9f9f9;}""")

        for a in [self.event_type, self.event_msg, self.robot_x, self.robot_y, self.timestamp]:
            a.setStyleSheet("font-size:16px")
            a.setMinimumHeight(40)

        self.event_img = QLabel()
        self.event_img.setFixedSize(560,300)
        self.event_img.setAlignment(Qt.AlignCenter)
        self.event_img.setScaledContents(True)
        self.event_img.setStyleSheet("""QLabel {background-color: #eeeeee; border: 1px solid #999999;}""")

        event_layout = QVBoxLayout()
        event_layout.setContentsMargins(20, 20, 20, 20)
        event_layout.setSpacing(15)
        event_layout.addWidget(self.event_type)
        event_layout.addWidget(self.event_msg)
        event_layout.addWidget(self.robot_x)
        event_layout.addWidget(self.robot_y)
        event_layout.addWidget(self.timestamp)
        event_layout.addWidget(self.event_img)
        event_layout.addStretch()
        self.event_frame.setLayout(event_layout)

        self.btn_log = QPushButton('Log History')
        self.btn_log.setFixedHeight(70)
        self.btn_log.setStyleSheet("""QPushButton {background-color: #ecf0f1; font-size: 18px; font-weight: bold; border: 2px solid #000000; border-radius: 8px;}
            QPushButton:hover {background-color: #d5dbdb;} QPushButton:pressed {background-color: #7f8c8d;}""")

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.event_frame)
        right_layout.addWidget(self.btn_log)

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

        self.camera_frame = None
        self.map_image = None
        self.map_resolution = None
        self.map_origin = None
        self.path_points = []
        self.robot_pose = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)
        self.log_window = None

    def btn_connect(self, ros):
        self.btn_drive.clicked.connect(ros.send_start)
        self.btn_stop.clicked.connect(ros.send_stop)
        self.btn_auto.clicked.connect(ros.auto)
        self.btn_manual.clicked.connect(ros.manual)
        self.btn_emerge.clicked.connect(ros.send_emergstop)
        self.btn_log.clicked.connect(self.open_log_history)

    # 로봇의 월드 좌표를 지도 이미지 좌표로 변환
    def world_to_map(self, x, y):
        if (self.map_origin is None or self.map_resolution is None or
            self.map_image is None or self.grid_w is None or self.grid_h is None):
            return None

        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)

        if mx < 0 or my < 0 or mx >= self.grid_w or my >= self.grid_h:
            return None

        px = (self.grid_h - 1 - my)
        py = (self.grid_w - 1 - mx)

        if px < 0 or py < 0 or px >= self.map_image.shape[1] or py >= self.map_image.shape[0]:
            return None

        return (px, py)

    # 카메라 영상과 지도를 갱신하여 GUI 화면에 실시간으로 표시
    def update_frame(self):
        if self.camera_frame is not None:
            cam = cv2.resize(self.camera_frame, (600, 480))
            cv2.rectangle(cam, (0, 0), (599, 479), (0, 0, 0), 3)

            cam_rgb = cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)
            cam_img = QImage(
                cam_rgb.data,
                600, 480,
                3 * 600,
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

            m = cv2.resize(m, (600, 480))
            cv2.rectangle(m, (0, 0), (599, 479), (0, 0, 0), 3)

            map_rgb = cv2.cvtColor(m, cv2.COLOR_BGR2RGB)
            map_img = QImage(
                map_rgb.data,
                600, 480,
                3 * 600,
                QImage.Format_RGB888
            )
            self.map_label.setPixmap(QPixmap.fromImage(map_img))

    
    # 저장된 이벤트 로그를 확인할 수 있는 로그 히스토리 창
    def open_log_history(self):
        if self.log_window is None:
            self.log_window = LogHistoryWindow()

        self.log_window.show()
        self.log_window.raise_()
        self.log_window.activateWindow()

# 로그 히스토리 관련 GUI
class LogHistoryWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Log History")
        self.resize(1000, 600)

        layout = QVBoxLayout(self)

        self.table = QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["상황", "상태", "X", "Y", "이미지", "시간" ])

        header = self.table.horizontalHeader()
        self.table.setColumnWidth(0, 180) 
        self.table.setColumnWidth(1, 180)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents) 
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)  
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents) 
        header.setSectionResizeMode(5, QHeaderView.Stretch)

        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)

        layout.addWidget(self.table)

        self.load_logs()
        self.setLayout(layout)

    
    # 데이터베이스에 저장된 이벤트 로그를 불러와 테이블에 표시
    def load_logs(self):
        logs = fetch_event_logs(limit=100)
        self.table.setRowCount(len(logs))

        for row, log in enumerate(logs):
            self.table.setItem(row, 0, QTableWidgetItem(log['situation']))
            self.table.setItem(row, 1, QTableWidgetItem(log['status']))
            self.table.setItem(row, 2, QTableWidgetItem(f"{log['x']:.2f}"))
            self.table.setItem(row, 3, QTableWidgetItem(f"{log['y']:.2f}"))
            self.table.setItem(row, 5, QTableWidgetItem(str(log['created_at'])))

            NORMAL_COLOR = QColor(255, 255, 255)
            HUMAN_COLOR     = QColor(255, 200, 200)
            EMERGENCY_COLOR = QColor(255, 245, 200)

            if "HUMAN" in log['situation']:
                bg_color = HUMAN_COLOR
            elif "EMERGENCY" in log['situation']:
                bg_color = EMERGENCY_COLOR
            else:
                bg_color = NORMAL_COLOR

            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                if item:
                    item.setBackground(bg_color)

            btn = QPushButton("보기")
            btn.clicked.connect(
                lambda _, url=log['image_path']: self.open_image(url)
            )
            self.table.setCellWidget(row, 4, btn)

    # 선택한 이벤트의 이미지 URL을 웹 브라우저로 열 수 있음
    def open_image(self, url):
        if url:
            webbrowser.open(url)

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


