import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_relative_move_msgs.action import Rotate, Translate

class MinimumJerkRelativeMove(Node):
    def __init__(self):
        super().__init__('minimal_jerk_relative_move')
        self._publisher = self.create_publisher(Twist, 'cmd_vel',10)
        self._listener_odom = self.create_subscription(Odometry, 'odom', self.listener_odom_callback, 10)
        self._action_rotation_server = ActionServer(self, Rotate, 'rotate', self.rotation_callback)
        self._action_translation_server = ActionServer(self, Translate, 'translate', self.translation_callback)

    def listener_odom_callback(self, msg):
        self.get_logger().info('Odom')
        m = Twist()
        self._publisher.publish(m)

    def rotation_callback(self, goal_handle):
        self.get_logger().info('Rotation')
        return Rotate.Result()

    def translation_callback(self, goal_handle):
        self.get_logger().info('Translation')
        return Translate.Result()
        

    