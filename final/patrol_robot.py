from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Int32
from patrol_msgs.msg import Event
from math import pow, atan2, sqrt, sin, pi, cos
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
from turtlebot3_msgs.srv import Sound

import rclpy
import heapq
import numpy as np
import os
import yaml

# A* 경로 탐색에서 사용하는 노드 정보를 저장
class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

 # 로봇의 전체 주행, 상태 관리, 센서/토픽/타이머 초기화
 # 순찰, 사람 추종, 신호등, 비상정지, 경로계획 등 통합 관리
class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        self.front_dist = 99.9
        self.left_dist = 99.9
        self.right_dist = 99.9

        self.lookahead_dist = 0.8
        self.linear_vel = 0.18
        self.robot_radius = 0.26

        self.aligning = False
        self.target_yaw = 0.0
        self.next_wp = None

        self.stop_tolerance = 0.3
        
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.robot_radius_cells = int(self.robot_radius / self.map_resolution)
        
        self.current_pose = None
        self.global_path = []
        self.current_yaw = 0.0
        self.path_index = 0

        self.waypoints = []
        self.wp_direction = 1
        self.wp_index = 0
        self.return_home = False
        self.return_wp_index = None
        self.current_goal_index = None
        self.first_start = True
        self.load_waypoints()

        self.stop_requested = False
        self.patrol_active = True
        self.emerge_stop_com = False

        self.mode = "MANUAL"
        self.traffic_state = "NO"
        self.human_state = "NONE"
        self.localization_ok = False
        self.signal_stop =  False

        self.image_center = 208 
        self.person = None
        self.human_detect = False

        self.buzzer_timer = None

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
        self.sub_traffic_msg = self.create_subscription(String, '/traffic_state', self.traffic_callback, 10)
        self.sub_human_msg = self.create_subscription(String, '/human_state', self.human_callback, 10)
        self.sub_person = self.create_subscription(Int32, '/person', self.person_callback, 10)

        self.buzzer = self.create_client(Sound, '/sound')


        self.timer = self.create_timer(0.3, self.control)
        self.wp_timer = self.create_timer(1.0, self.send_next_waypoint)
        self.get_logger().info("Let's Run!")

    # OccupancyGrid 맵을 수신해서 해상도, 크기, 원점, 맵 데이터 저장
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    # AMCL 위치를 받아 현재 로봇 위치(x, y)와 yaw 각도 계산
    # 주행 제어와 경로 추종에 사용
    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    # 목표 지점을 받아서 A* 알고리즘으로 경로 생성
    # 월드 좌표를 그리드 좌표로 변환 후 경로 생성
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
    
    # 라이다 데이터를 받아서 최소 장애물 거리 계산
    # 장애물 회피 로직을 구현할 때 사용
    def scan_callback(self,msg):
        n = len(msg.ranges)
        front = msg.ranges[0:12] + msg.ranges[-12:]
        left  = msg.ranges[15:40] 
        right = msg.ranges[n-40:n-15]    

        self.front_dist = self.get_min_dist(front)
        self.left_dist = self.get_min_dist(left)
        self.right_dist = self.get_min_dist(right)

    # 순찰 정지 명령을 구독해서 명령을 받으면 현재 순찰 중단
    def stop_callback(self, msg):
        if msg.data:
            self.stop_requested = True
            self.get_logger().info("Stop command!")

    # 비상정지 명령을 구독해서 명령을 받으면 즉시 정지
    # 토글 방식을 이용하여 한번더 명령을 받는다면 비상정지 해제
    # 현재 경로와 무관하게 즉시 정지
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

    # 순찰 시작/재개 명령을 구독해서 명령을 받으면 AUTO 모드에서 순찰 시작
    # 초기 시작과 재개를 구분
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

        if self.first_start:
            self.wp_index = 0
            self.first_start = False
        else:
            self.get_logger().info(f"Resume patrol (goal : wp_index={self.wp_index})")

    # AUTO 명령을 구독해서 명령을 받으면 자율 주행 활성화
    def auto_callback(self,msg):
        if msg.data:
            self.mode = "AUTO"
            self.patrol_active = True
            self.localization_ok = True
            self.human_detect = False
            self.signal_stop = False
            self.global_path = []
            self.get_logger().info("AUTO mode")

    # MANUAL 명령을 구독해서 명령을 받으면 수동 모드로 전환
    # 로봇이 움직이는 중이면 전환 금지
    def manual_callback(self,msg):
        if msg.data:
            if self.global_path:
                self.get_logger().info("Can't Switch. Robot is Moving")
                return
            self.mode = "MANUAL"
            self.localization_ok = False
            self.get_logger().info("MANUAL mode")

    # 초기 위치 설정 신호를 받아 로컬라이제이션 완료 상태로 만듬
    def signal_callback(self,msg):
        if not self.localization_ok:
            self.localization_ok = True

    # 신호등 상태를 받아서 정지 여부 결정
    def traffic_callback(self,msg):
        self.traffic_state = msg.data

        if self.traffic_state == "RED":
            self.signal_stop = True

        elif self.traffic_state == "GREEN":
            self.signal_stop = False

    # 사람 감지 상태를 받아서 사람 추종/경로 복귀 제어
    # 사람이 감지 되면 부저 발생
    def human_callback(self,msg):
        prev = self.human_state
        self.human_state = msg.data

        if prev == "NONE" and self.human_state == "DETECT":
            self.human_detect = True

            self.global_path = []
            self.path_index = 0

            self.event("HUMAN DETECT", "FOLLOW HUMAN")
            self.start_buzzer()

        if prev == "DETECT" and self.human_state == "NONE":
            self.person = None
            self.human_detect = False
            self.patrol_active = True 

            self.stop_buzzer()

            self.global_path = []
            self.path_index = 0

            wp = self.waypoints[self.wp_index]
            msg = PoseStamped()
            msg.pose.position.x = wp['x']
            msg.pose.position.y = wp['y']
            msg.pose.orientation.w = 1.0

            self.goal_callback(msg)

            if not self.global_path:
                self.get_logger().warn("Return path failed")

    # 사람의 화면 중심 위치를 받아서 추종 제어(위치 기준 로봇이 사람을 따라 가도록)에 사용
    def person_callback(self,msg):
        self.person = int(msg.data)

    # 부저 재생
    def buzzer_callback(self):
        self.play_sound(2)

    # 현재 로봇 상태를 메세지로 발행
    # 특수 상태 2개를 발행
    # 사람 감지, 비상 정지
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

    # A* 알고리즘을 사용해서 시작점에서 목표점까지 경로 계산
    # 로봇 반경을 고려해 장애물과 너무 가까운 경로 제외
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
                    
                cost = sqrt(2) if move[0] != 0 and move[1] != 0 else 1.0
                new_node = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + cost
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    # 주행 로직의 메인 제어 루프
    # 비상정지, 신호등 제어, 사람/경로 추종 제어 상태를 판단하여 동작 결정
    def control(self):
        if self.emerge_stop_com:
            self.stop_robot()
            return

        if not self.localization_ok:
            return

        if self.mode == "MANUAL":
            return

        if self.human_state == "PERSON":
            if self.front_dist < 0.4:
                self.stop_robot()
                return
            if self.signal_stop is False and self.emerge_stop_com is False:
                self.follow_human()
                return
            return
        
        if self.aligning:
            self.turn_align()
            return

        if self.signal_stop:
            self.stop_robot()
            return

        if not self.global_path:
            return

        self.follow_path()

    def turn_align(self):
        if self.aligning:
            yaw_error = self.target_yaw - self.current_yaw
            if yaw_error > pi:
                yaw_error -= 2 * pi
            elif yaw_error < -pi:
                yaw_error += 2 * pi

            cmd = Twist()

            if abs(yaw_error) > 0.1:
                k = 1.2
                cmd.angular.z = k * yaw_error

                if cmd.angular.z > 0.8:
                    cmd.angular.z = 0.8
                elif cmd.angular.z < -0.8:
                    cmd.angular.z = -0.8
                    
                cmd.linear.x = 0.0
                self.pub_cmd.publish(cmd)
                return
            else:
                self.aligning = False
                self.wp_index = self.next_wp   
                self.next_wp = None
                self.stop_robot()
                return

    # A* 알고리즘으로 생성된 경로를 따라서 로봇을 이동
    # lookahead 기반으로 목표점 선택
    def follow_path(self):
        if self.current_pose is None or not self.global_path:
            return

        final_goal = self.global_path[-1]

        dist_to_final = sqrt(
            (final_goal[0] - self.current_pose[0]) ** 2 +
            (final_goal[1] - self.current_pose[1]) ** 2
        )

        if dist_to_final < self.stop_tolerance:
            self.global_path = []
            self.path_index = 0

            arrived_wp = self.wp_index

            if self.stop_requested and arrived_wp == 0:
                self.patrol_active = False
                self.stop_requested = False
                self.stop_robot()
                return

            next_wp = arrived_wp + self.wp_direction

            if next_wp >= len(self.waypoints):
                next_wp = len(self.waypoints) - 2
                self.wp_direction = -1
            elif next_wp < 0:
                next_wp = 1
                self.wp_direction = 1

            self.next_wp = next_wp

            curr = self.current_pose
            wp = self.waypoints[self.next_wp]
            dx = wp['x'] - curr[0]
            dy = wp['y'] - curr[1]
            self.target_yaw = atan2(dy, dx)

            self.aligning = True
            self.stop_robot()
            return


        target_x, target_y = final_goal

        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt(
                (px - self.current_pose[0]) ** 2 +
                (py - self.current_pose[1]) ** 2
            )

            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i
                break

        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]

        alpha = atan2(dy, dx) - self.current_yaw

        if alpha > pi:
            alpha -= 2 * pi
        elif alpha < -pi:
            alpha += 2 * pi

        cmd = Twist()

        cmd = self.avoid_obstacle(cmd, alpha)
        if cmd is None:
            return

        self.pub_cmd.publish(cmd)

    # 카메라 센서 기반 사람 위치를 사용하여 사람 추종 
    # 화면 중심 오차를 이용해 속도 제어
    def follow_human(self):
        if self.person is None:
            self.stop_robot()
            return

        if self.emerge_stop_com:
            self.stop_robot()
            return

        error = float(self.person - self.image_center)

        cmd = Twist()
        cmd.linear.x = 0.18
        cmd.angular.z = -0.003 * error

        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0

        cmd = self.avoid_obstacle(cmd, alpha=0.0)

        if cmd is None:
            return

        self.pub_cmd.publish(cmd)

    # 라이다 센서를 통해 얻은 거리를 기반으로 장애물 회피
    # 거리 조건에 따라 속도 조절
    def avoid_obstacle(self, cmd, alpha):
        target_linear = self.linear_vel
        target_angular = target_linear * self.linear_vel * sin(alpha) / self.lookahead_dist

        avoid_dir = 0.0
        speed_mag = 1.0

        if self.front_dist < 0.5:
            if self.left_dist >= self.right_dist:
                avoid_dir = 0.5
            else:
                avoid_dir = -0.5

            speed_mag = 0.2

        elif self.left_dist < 0.3:
            avoid_dir = -0.5
            speed_mag = min(speed_mag, 0.2)

        elif self.right_dist < 0.3:
            avoid_dir = 0.5
            speed_mag = min(speed_mag, 0.2)

        if self.front_dist < 0.25:
            self.stop_robot()
            return

        cmd.linear.x = target_linear * speed_mag
        cmd.angular.z = target_angular + avoid_dir

        if cmd.angular.z > 1.2:
            cmd.angular.z = 1.2
        elif cmd.angular.z < -1.2:
            cmd.angular.z = -1.2

        return cmd

    # YAML 파일에서 순찰 웨이포인트(경유지)를 불러옴
    def load_waypoints(self):
        yaml_path = '/home/cho/lch_ws/src/turtle_pkg/config/patrol_waypoints.yaml'
        if not os.path.exists(yaml_path):
            self.get_logger().error('Waypoint YAML not found')
            return

        with open(yaml_path, 'r') as f:
            self.waypoints = yaml.safe_load(f)

        self.get_logger().info(f'{len(self.waypoints)} waypoints loaded')

    # 현재 상태에 따라 다음 웨이포인트로 이동 명령 보냄
    # 순찰, 복귀, 정렬 상태 등을 고려함
    def send_next_waypoint(self):
        if self.aligning:
            return
            
        if not self.waypoints:
            return

        if self.human_detect:
            return

        if self.map_data is None or self.current_pose is None:
            return

        if self.global_path:
            return

        if self.return_home:
            wp = self.waypoints[self.return_wp_index] 

            msg = PoseStamped()
            msg.pose.position.x = wp['x']
            msg.pose.position.y = wp['y']
            msg.pose.orientation.z = sin(wp['yaw'] / 2.0)
            msg.pose.orientation.w = cos(wp['yaw'] / 2.0)

            self.goal_callback(msg)

            self.return_wp_index -= 1

            if self.return_wp_index < 0:
                self.stop_robot()
                self.return_home = False
                self.stop_requested = False
                self.patrol_active = False
            return

        if not self.patrol_active:
            return

        self.current_goal_index = self.wp_index

        wp = self.waypoints[self.wp_index]
        msg = PoseStamped()
        msg.pose.position.x = wp['x']
        msg.pose.position.y = wp['y']
        msg.pose.orientation.z = sin(wp['yaw'] / 2.0)
        msg.pose.orientation.w = cos(wp['yaw'] / 2.0)

        self.goal_callback(msg)

    # 월드 좌표를 그리드 좌표로 변환
    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    # 그리드 좌표를 월드 좌표로 변환
    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0],
                (grid[0]*self.map_resolution)+self.map_origin[1]]

    # 계산된 글로벌 경로를 GUI에서 시각화하기 위해 Path 토픽으로 발행
    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    # 라이다 센서를 통해 얻은 거리 값에서 유효한 최소 거리 계산
    def get_min_dist(self, range_list):

        valid_values = [x for x in range_list if x > 0.05 and x < 10.0]
        if valid_values:
            return min(valid_values)
        return 99.9

    # 터틀봇 부저 사운드 재생
    def play_sound(self, value: int):
        req = Sound.Request()
        req.value = int(value)
        self.buzzer.call_async(req)

    # 사람 감지 시 1초 간격으로 부저 울림
    def start_buzzer(self):
        if self.buzzer_timer is not None:
            return
        self.buzzer_timer = self.create_timer(1.0, self.buzzer_callback)

    # 부저 타이머를 중지해서 부저 멈추기
    def stop_buzzer(self):
        if self.buzzer_timer is not None:
            self.buzzer_timer.cancel()
            self.buzzer_timer = None

    # 로봇을 즉시 정지시키기 위하여 0 속도 Twist 발행
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