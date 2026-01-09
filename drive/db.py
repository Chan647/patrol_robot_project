import rclpy
from rclpy.node import Node
from patrol_msgs.msg import Event
import pymysql

class DBLogger(Node):
    def __init__(self):
        super().__init__('db_logger')

        self.sub = self.create_subscription(Event, '/patrol_event', self.callback, 10)

        self.conn = pymysql.connect(
            host='localhost',
            user='robot',
            password='robot1234',
            database='robot_db',
            charset='utf8mb4'
        )

    def callback(self, msg):
        with self.conn.cursor() as cursor:
            sql = """
            INSERT INTO situation_log
            (situation, status, x, y, image_path)
            VALUES (%s, %s, %s, %s, %s)
            """
            cursor.execute(
                sql,
                (msg.situation, msg.status, msg.x, msg.y, msg.image_path)
            )
        self.conn.commit()
