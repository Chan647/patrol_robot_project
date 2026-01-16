import rclpy
import cv2
import numpy as np
import joblib

from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node



qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class PersonDetectNode(Node):
    def __init__(self):
        super().__init__('person_detect_node')

        self.model = YOLO('/home/cho/lch_ws/src/turtle_pkg/turtle_pkg/patrol_robot/yolov8n.pt')
        self.model_data = joblib.load('/home/cho/lch_ws/src/turtle_pkg/config/person_gate_model.joblib')
        
        self.pub_person = self.create_publisher(Int32, '/person', 10)
        self.pub_state = self.create_publisher(String,'/human_state', 10)

        self.sub_img = self.create_subscription(CompressedImage,'/image_raw/compressed', self.image_callback, qos)

        self.time = self.get_clock().now()
        self.detect_time = self.get_clock().now()

        self.detect_timeout = 0.6
        self.resize_width = 416
        self.process_timeout = 0.3

        self.ml_model = self.model_data["model"]
        self.feature_cols = self.model_data["feature_cols"]
        self.ml_threshold = 0.7


    def image_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.time).nanoseconds < int(self.process_timeout * 1e9):
            return
        self.time = now

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w, _ = frame.shape
        if w != self.resize_width:
            scale = self.resize_width / float(w)
            frame = cv2.resize(frame, (self.resize_width, int(h * scale)))

        frame_h, frame_w, _ = frame.shape
        frame_area = frame_h * frame_w

        detected = False
        best_center = None
        best_prob = 0.0

        results = self.model(frame, conf=0.3, verbose=False)

        for r in results:
            if r.boxes is None:
                continue

            for b in r.boxes:
                cls = int(b.cls[0])
                conf = float(b.conf[0])

                if cls != 0:
                    continue

                x1, y1, x2, y2 = b.xyxy[0].cpu().numpy().astype(int)
                w_box = x2 - x1
                h_box = y2 - y1

                if w_box <= 0 or h_box <= 0:
                    continue

                area = w_box * h_box
                aspect_ratio = h_box / float(w_box)

                if area < frame_area * 0.03:
                    continue
                if aspect_ratio < 1.3:
                    continue

                center_x = int((x1 + x2) / 2)

                feature = np.array([[
                    conf,                              
                    center_x / float(frame_w),          
                    area / float(frame_area),          
                    aspect_ratio,                    
                    w_box / float(frame_w),            
                    h_box / float(frame_h),          
                ]], dtype=np.float32)


                prob = self.ml_model.predict_proba(feature)[0, 1]

                if prob > self.ml_threshold and prob > best_prob:
                    best_prob = prob
                    best_center = center_x
                    detected = True

        state_msg = String()
        person_msg = Int32()

        if detected:
            self.detect_time = now
            state_msg.data = "DETECT"
            self.pub_state.publish(state_msg)

            person_msg.data = best_center
            self.pub_person.publish(person_msg)

        else:
            if (now - self.detect_time).nanoseconds < int(self.detect_timeout * 1e9):
                return

            state_msg.data = "NONE"
            self.pub_state.publish(state_msg)



def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()