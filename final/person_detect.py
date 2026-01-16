import rclpy
import cv2
import numpy as np
import pandas as pd
import os
from datetime import datetime

from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Int32, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

class PersonDetectNode(Node):
    def __init__(self):
        super().__init__('person_detect_node')

        self.model = YOLO(
            '/home/cho/lch_ws/src/turtle_pkg/turtle_pkg/patrol_robot/yolov8n.pt'
        )

        self.pub_state  = self.create_publisher(String,  '/human_state', 10)
        self.pub_center = self.create_publisher(Int32,   '/person', 10)
        self.pub_conf   = self.create_publisher(Float32, '/person_conf', 10)

        self.sub_img = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos
        )

        self.resize_width = 416
        self.conf_threshold = 0.6
        self.detect_timeout = 0.6

        self.last_detect_time = self.get_clock().now()
        self.last_process_time = self.get_clock().now()

        self.csv_path = '/home/cho/lch_ws/src/turtle_pkg/config/person_dataset.csv'
        self.columns = [
            "yolo_conf",
            "center_x_norm",
            "area_ratio",
            "aspect_ratio",
            "w_ratio",
            "h_ratio",
            "label"
        ]

        if os.path.exists(self.csv_path):
            self.df = pd.read_csv(self.csv_path)
        else:
            self.df = pd.DataFrame(columns=self.columns)

    def image_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_process_time).nanoseconds < 300_000_000:
            return
        self.last_process_time = now

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w, _ = frame.shape
        scale = self.resize_width / float(w)
        frame = cv2.resize(frame, (self.resize_width, int(h * scale)))

        frame_h, frame_w, _ = frame.shape
        frame_area = frame_h * frame_w

        best_score = 0.0
        best_center = None
        best_conf = 0.0
        best_feat = None

        results = self.model(frame, conf=0.3, verbose=False)

        for r in results:
            if r.boxes is None:
                continue

            for b in r.boxes:
                cls  = int(b.cls[0])
                conf = float(b.conf[0])

                if cls != 0:
                    continue
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = b.xyxy[0].cpu().numpy().astype(int)
                w_box = x2 - x1
                h_box = y2 - y1
                area = w_box * h_box
                aspect_ratio = h_box / float(w_box + 1e-6)

                if area < frame_area * 0.03:
                    continue
                if aspect_ratio < 1.3:
                    continue

                score = conf * area
                if score > best_score:
                    best_score = score
                    best_center = int((x1 + x2) / 2)
                    best_conf = conf

                    best_feat = {
                        "yolo_conf": conf,
                        "center_x_norm": best_center / frame_w,
                        "area_ratio": area / frame_area,
                        "aspect_ratio": aspect_ratio,
                        "w_ratio": w_box / frame_w,
                        "h_ratio": h_box / frame_h,
                        "label": 1   
                    }

        state_msg = String()

        if best_center is not None:
            self.last_detect_time = now

            state_msg.data = "PERSON"
            self.pub_state.publish(state_msg)

            self.pub_center.publish(Int32(data=best_center))
            self.pub_conf.publish(Float32(data=best_conf))

            self.df = pd.concat(
                [self.df, pd.DataFrame([best_feat])],
                ignore_index=True
            )
            self.df.to_csv(self.csv_path, index=False)

        else:
            if (now - self.last_detect_time).nanoseconds < int(self.detect_timeout * 1e9):
                return

            state_msg.data = "NORMAL"
            self.pub_state.publish(state_msg)

def main():
    rclpy.init()
    node = PersonDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
