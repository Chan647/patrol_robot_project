import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt, sin, pi, cos
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from patrol_msgs.msg import Event
import cv2
import heapq
import numpy as np
import os
import yaml

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        self.lookahead_dist = 0.5
        self.linear_vel = 0.12
        self.stop_tolerance = 0.3
        self.robot_radius = 0.24
        
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.robot_radius_cells = int(self.robot_radius / self.map_resolution)
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = []
        self.path_index = 0
        
        self.front_dist = 99.9

        self.pose_count = 0
        self.waypoints = []
        self.wp_index = 0
        self.load_waypoints()

        self.stop_requested = False
        self.current_goal_is_home = False
        self.home_pose = None
        self.patrol_active = True
        self.emerge_stop_com = False

        self.mode = "MANUAL"
        self.localization_ok = False
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        self.pub_event = self.create_publisher(Event, '/patrol_event', 10)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sub_stop = self.create_subscription(Bool, '/patrol_stop', self.stop_callback, 10)
        self.sub_emrgestop = self.create_subscription(Bool, '/emergency_stop', self.emerge_callback, 10)
        self.sub_start = self.create_subscription(Bool, '/patrol_start', self.start_callback, 10)
        self.sub_auto = self.create_subscription(Bool, '/auto_mode', self.auto_callback, 10)
        self.sub_manual = self.create_subscription(Bool, '/manual_mode', self.manual_callback, 10)
        self.sub_signal = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.signal_callback, 10)

        self.timer = self.create_timer(0.3, self.control_loop)
        self.wp_timer = self.create_timer(1.0, self.send_next_waypoint)
        self.get_logger().info("Let's Run!")

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.pose_count += 1
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

        if not hasattr(self, 'home_pose') or self.home_pose is None:
            self.home_pose = {'x': self.current_pose[0], 'y': self.current_pose[1], 'yaw': self.current_yaw}

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None: return

        goal_pose = [msg.pose.position.x, msg.pose.position.y]
        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)
        path_grid = self.run_astar(start_grid, goal_grid)
        
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
        else:
            self.get_logger().warn("No Path Found.")    

    def scan_callback(self,msg):
        front_ranges = msg.ranges[0:15] + msg.ranges[-15:]

        self.front_dist = self.get_min_dist(front_ranges)

    def stop_callback(self, msg):
        if msg.data:
            self.stop_requested = True
            self.patrol_active = False
            self.current_goal_is_home = False
            self.get_logger().info("Stop command!")

    def emerge_callback(self, msg):
        if not msg.data:
            return

        self.emerge_stop_com = not self.emerge_stop_com

        if self.emerge_stop_com:
            self.stop_robot()
            self.event("EMERGENCY STOP", "ROBOT STOP")
            self.get_logger().warn("Emergency Stop!!")
        else:
            self.get_logger().info("Emergency Stop Released")

    def start_callback(self, msg):
        if not msg.data:
            return

        if self.mode != "AUTO":
            return

        if not self.localization_ok:
            return

        if self.emerge_stop_com:
            return

        self.patrol_active = True
        self.stop_requested = False
        self.global_path = []
        self.path_index = 0
        self.wp_index = 0
        self.get_logger().info("Resuming patrol.")  

    def auto_callback(self,msg):
        if msg.data:
            if not self.localization_ok:
                return
            self.mode = "AUTO"
            self.patrol_active = True 
            self.get_logger().info("AUTO mode")

    def manual_callback(self,msg):
        if msg.data:
            if self.global_path:
                self.get_logger().info("Can't Switch. Robot is Moving")
                return
            self.mode = "MANUAL"
            self.localization_ok = False
            self.get_logger().info("MANUAL mode")

    def signal_callback(self,msg):
        if not self.localization_ok:
            self.localization_ok = True

    def event(self, situation, status, image_path=""):
        if self.current_pose is None:
            return

        msg = Event()
        msg.situation = situation
        msg.status = status
        msg.x = float(self.current_pose[0])
        msg.y = float(self.current_pose[1])
        msg.image_path = image_path

        self.pub_event.publish(msg)

    def run_astar(self, start, end):
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0] < self.map_height and 0 <= end[1] < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []
        heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            for move in moves:
                ny, nx = current_node.position[0] + move[0], current_node.position[1] + move[1]
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] != 0: continue

                too_close = False
                for dy in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                    for dx in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                        cy, cx = ny + dy, nx + dx
                        if 0 <= cy < self.map_height and 0 <= cx < self.map_width:
                            if self.map_data[cy][cx] != 0:
                                too_close = True
                                break
                    if too_close:
                        break

                if too_close:
                    continue
                
                new_node = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + 1
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    def control_loop(self):
        if self.emerge_stop_com:
            self.stop_robot()
            return

        if self.mode == "MANUAL":
            return

        if not self.global_path:
            return

        final_goal = self.global_path[-1]
        dist_to_final = sqrt((final_goal[0]-self.current_pose[0])**2 + (final_goal[1]-self.current_pose[1])**2)

        if self.emerge_stop_com:
            self.stop_robot()

        if dist_to_final < self.stop_tolerance:
            self.global_path = []

            if self.stop_requested and not self.current_goal_is_home:
                msg = PoseStamped()
                msg.pose.position.x = self.home_pose['x']
                msg.pose.position.y = self.home_pose['y']
                msg.pose.orientation.z = sin(self.home_pose['yaw'] / 2.0)
                msg.pose.orientation.w = cos(self.home_pose['yaw'] / 2.0)

                self.current_goal_is_home = True
                self.goal_callback(msg)
                return

            if self.current_goal_is_home:
               self.stop_robot()
               self.stop_requested = False
               self.current_goal_is_home = False
               self.get_logger().info("Returned to home position. Stop the robot")
               return

            return

        target_x, target_y = final_goal

        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px - self.current_pose[0])**2 + (py - self.current_pose[1])**2)

            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i
                break

        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        alpha = atan2(dy, dx) - self.current_yaw
        
        if alpha > pi: alpha -= 2*pi
        elif alpha < -pi: alpha += 2*pi

        cmd = Twist()

        
        if abs(alpha) > 1.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.4 if alpha > 0 else -0.4

        elif self.front_dist < 0.4:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        else:
            angular_velocity = self.linear_vel * (2.0 * sin(alpha)) / self.lookahead_dist
            cmd.linear.x = self.linear_vel
            cmd.angular.z = angular_velocity
        
        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0
        
        self.pub_cmd.publish(cmd)

    def load_waypoints(self):
        yaml_path = '/home/cho/lch_ws/src/turtle_pkg/config/patrol_waypoints.yaml'
        if not os.path.exists(yaml_path):
            self.get_logger().error('Waypoint YAML not found')
            return

        with open(yaml_path, 'r') as f:
            self.waypoints = yaml.safe_load(f)

        if self.waypoints:
            self.home_pose = self.waypoints[0]

        self.get_logger().info(f'{len(self.waypoints)} waypoints loaded')

    def send_next_waypoint(self):
        if not self.waypoints:
            return

        if self.map_data is None or self.current_pose is None:
            return

        if self.global_path:
            return

        if not self.patrol_active:
            return

        wp = self.waypoints[self.wp_index]
        msg = PoseStamped()
        msg.pose.position.x = wp['x']
        msg.pose.position.y = wp['y']
        msg.pose.orientation.z = sin(wp['yaw'] / 2.0)
        msg.pose.orientation.w = cos(wp['yaw'] / 2.0)

        self.current_goal_is_home = False
        self.goal_callback(msg)

        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.wp_index = 0

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0],
                (grid[0]*self.map_resolution)+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def get_min_dist(self, range_list):

        valid_values = [x for x in range_list if x > 0.05 and x < 10.0]
        if valid_values:
            return min(valid_values)
        return 99.9

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()