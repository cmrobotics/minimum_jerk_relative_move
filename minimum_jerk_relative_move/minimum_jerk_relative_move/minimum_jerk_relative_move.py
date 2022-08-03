from numpy import double
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from cmr_msgs.action import Rotate, Translate
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .mathematical_fonctions import euler_from_quaternion


class MinimumJerkRelativeMove(Node):

    def __init__(self):

        self._init_pose = {"x": 0, "y": 0, "angle": 0}
        self._current_pose = {"x": 0, "y": 0, "angle": 0}

        super().__init__('minimal_jerk_relative_move')
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._action_rotation_server = ActionServer(
            self, Rotate, 'rotate', self.rotation_callback)
        self._action_translation_server = ActionServer(
            self, Translate, 'translate', self.translation_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_footprint to odom: {ex}')
            return
        self._current_pose["x"] = trans.transform.translation.x
        self._current_pose["y"] = trans.transform.translation.x
        self._current_pose["angle"] = euler_from_quaternion(
            trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)["z"]

    def rotation_callback(self, goal_handle):
        self.get_logger().info('Rotation')
        t_init = time.time()
        self._init_pose = self._current_pose
        rotation_feedback = Rotate.Feedback()
        yaw_tolerance = 0.1
        if (goal_handle.request.yaw_goal_tolerance > 0):
            yaw_tolerance = goal_handle.request.yaw_goal_tolerance
        while (abs(self._current_pose["angle"] - goal_handle.request.target_yaw) < yaw_tolerance):
            rotation_feedback = self._current_pose["angle"] - self._init_pose["angle"]
            goal_handle.publish_feedback(rotation_feedback)

        goal_handle.succeed()
        res = Rotate.Result()
        res.total_elapsed_time.nanosec = int((time.time() - t_init) * 10**9)
        return res

    def translation_callback(self, goal_handle):
        self.get_logger().info('Translation')
        t_init = time.time()
        self._init_pose = self._current_pose
        translation_feedback = Translate.Feedback()
        xy_tolerance = 0.015
        if (goal_handle.request.xy_goal_tolerance > 0):
            xy_tolerance = goal_handle.request.xy_goal_tolerance
        while (abs(self._current_pose["x"] - goal_handle.request.target.x) < xy_tolerance):
            translation_feedback = self._current_pose["x"] - self._init_pose["x"]
            goal_handle.publish_feedback(translation_feedback)

        goal_handle.succeed()
        res = Translate.Result()
        res.total_elapsed_time.nanosec = int((time.time() - t_init) * 10**9)
        return res
