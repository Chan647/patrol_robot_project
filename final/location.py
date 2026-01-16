import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import yaml
import math

# 로봇의 시작 위치와 사용자 정의 웨이포인트를 지정해서 YAML파일로 저장하는 노드
class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')

        self.waypoints = []
        self.start_pose_added = False

        self.goal_sub = self.create_subscription( PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.amcl_sub = self.create_subscription( PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)


    def amcl_callback(self, msg):
        if self.start_pose_added:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        start_way = {
            'x': float(round(x, 3)),
            'y': float(round(y, 3)),
            'yaw': float(round(yaw, 3))
        }

        self.waypoints.append(start_way)
        self.start_pose_added = True

        self.get_logger().info(f'start waypoint add: {start_way}')

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        waypoint = {
            'x': float(round(x, 3)),
            'y': float(round(y, 3)),
            'yaw': float(round(yaw, 3))
        }

        self.waypoints.append(waypoint)
        self.get_logger().info(f'Waypoint add: {waypoint}')

    def save(self):
        save_path = '/home/cho/lch_ws/src/turtle_pkg/config/patrol_waypoints.yaml'

        with open(save_path, 'w') as f:
            yaml.dump(self.waypoints, f)

def main():
    rclpy.init()
    node = WaypointCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save()
        node.get_logger().info('complete waypoint save: patrol_waypoints.yaml')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
