import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_sub = self.create_subscription(Image,'/img',self.image_callback,10)

        self.frame = None
        self.get_logger().info('사람 추종 시작')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame = frame
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(str(e))

    def process_frame(self, frame):
        h, w = frame.shape[:2]
        frame_center_x = w / 2

        results = self.model(frame, conf=0.6)
        twist = Twist()
        person_detected = False

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls != 0:    # 사람만
                    continue

                person_detected = True

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                person_center_x = (x1 + x2) / 2
                error = person_center_x - frame_center_x
                area = (x2 - x1) * (y2 - y1)

                # 전진
                if area < 50000:
                    twist.linear.x = 0.2
                else:
                    twist.linear.x = 0.0

                # 회전
                twist.angular.z = -error / 200.0

                # 시각화
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame,'person',(x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 255, 0),2)
                break

        if not person_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        cv2.imshow('Person Follower', frame)
        cv2.waitKey(30)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
