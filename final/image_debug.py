import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy


qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class VisionDebugNode(Node):
    def __init__(self):
        super().__init__('vision_debug_node')

        self.traffic_state = "NO"
        self.human_state = "NONE"
        self.person = None

        self.sub_img = self.create_subscription(CompressedImage,'/image_raw/compressed',self.image_callback, qos)
        self.sub_traffic = self.create_subscription(String, '/traffic_state', self.traffic_callback, 10)
        self.sub_human = self.create_subscription(String, '/human_state', self.human_callback, 10)
        self.sub_person = self.create_subscription(Int32, '/person', self.person_callback,10)

        self.pub_debug = self.create_publisher(CompressedImage,'/image_debug', 10)

    def traffic_callback(self, msg):
        self.traffic_state = msg.data

    def human_callback(self, msg):
        self.human_state = msg.data
        if self.human_state != "PERSON":
            self.person = None

    def person_callback(self, msg):
        self.person = int(msg.data)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w, _ = frame.shape
        center = w // 2

        traffic_color = (200, 200, 200)
        human_color = (255, 255, 0)

        if self.traffic_state == "RED":
            traffic_color = (0, 0, 255)
        elif self.traffic_state == "GREEN":
            traffic_color = (0, 255, 0)

        if self.human_state == "NONE":
            human_color = (180, 180, 180)

        cv2.putText(frame, f"TRAFFIC: {self.traffic_state}", (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, traffic_color, 2)

        cv2.putText(frame, f"HUMAN: {self.human_state}", (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, human_color, 2)

        success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not success:
            return

        out = CompressedImage()
        out.format = 'jpeg'
        out.data = encoded.tobytes()

        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
