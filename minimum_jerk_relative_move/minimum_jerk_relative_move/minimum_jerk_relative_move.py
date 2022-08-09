import rclpy
import time
from rclpy.time import Time, Duration
from rclpy.clock import Clock
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMsg
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from cmr_msgs.action import Rotate, Translate
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from minimum_jerk_trajectory_planner.TrajectoryPlanners import MinimumJerkTrajectoryPlanner
from minimum_jerk_trajectory_planner.Pose import Pose
from minimum_jerk_trajectory_planner.Robot import Robot
from .mathematical_fonctions import distance, euler_from_quaternion

dt = 0.1


class MinimumJerkRelativeMove(Node):

    def __init__(self):
        super().__init__('minimum_jerk_relative_move')
        self.get_logger().info('Initialized Minimum Jerk relative move Node')

        self._init_pose = Pose(0, 0, 0)
        self._current_pose = Pose(0, 0, 0)

        self._publisher = self.create_publisher(
            Twist, 'cmd_vel', qos_profile_system_default)
        self._action_rotation_server = ActionServer(
            self, Rotate, 'rotate', self.rotation_callback)
        self._action_translation_server = ActionServer(
            self, Translate, 'translate', self.translation_callback)
        self.clock_subscription = self.create_subscription(
            ClockMsg, "clock", self.clock_callback, qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.use_sim_time_ = self.get_parameter("use_sim_time").value
        self.sim_time_stamp_ = Time().to_msg()

    """
    Get simulation clock
    """

    def clock_callback(self, msg):
        self.sim_time_stamp_ = msg.clock

    def get_current_pose(self):
        try:
            if (self.use_sim_time_):
                now = self.sim_time_stamp_
            else:
                now = Time()

            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform base_footprint to odom: {ex}')
            return None

        pose = Pose()
        pose.x = trans.transform.translation.x
        pose.y = trans.transform.translation.y
        pose.theta = euler_from_quaternion(
            trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)["z"]

        return pose

    def rotation_callback(self, goal_handle):
        self.get_logger().info('Rotation')

        t_init = time.time()

        self._init_pose = self.get_current_pose()
        if (self._init_pose is None):
            goal_handle.abort()

        controller = MinimumJerkTrajectoryPlanner()
        pose_target = Pose(0, 0, goal_handle.request.target_yaw)
        pose_start = Pose(0, 0, 0)

        dist = abs(pose_target.theta)
        max_total = dist/goal_handle.request.min_rotational_vel
        robot = Robot("Robot", "b", max_total, controller,
                      pose_start, pose_target, "r")
        robot.generate_trajectory(dt)

        rotation_feedback = Rotate.Feedback()
        angular_vel = Twist()
        angular_vel.angular = Vector3()

        yaw_tolerance = 0.1
        if (goal_handle.request.yaw_goal_tolerance > 0):
            yaw_tolerance = goal_handle.request.yaw_goal_tolerance

        for vel in robot.odometry.velocities:
            # Ugly way of accepting cancelation, use callbacks later
            if (goal_handle.is_cancel_requested):
                angular_vel.angular.z = 0.0
                self._publisher.publish(angular_vel)
                goal_handle.canceled()

            self._current_pose = self.get_current_pose()
            if (self._current_pose is None):
                goal_handle.abort()

            rotation_feedback.angular_distance_traveled = abs(
                self._current_pose.theta - self._init_pose.theta)
            angular_vel.angular.z = float(vel.theta)
            self._publisher.publish(angular_vel)
            goal_handle.publish_feedback(rotation_feedback)
            time.sleep(dt)

        angular_vel.angular.z = 0.0
        self._publisher.publish(angular_vel)

        self._current_pose = self.get_current_pose()
        if (self._current_pose is None):
            goal_handle.abort()
        rotation_feedback.angular_distance_traveled = abs(
            self._current_pose.theta - self._init_pose.theta)

        if abs(rotation_feedback.angular_distance_traveled - abs(goal_handle.request.target_yaw)) <= yaw_tolerance:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        res = Rotate.Result()
        res.total_elapsed_time.sec = int((time.time() - t_init))
        return res

    def translation_callback(self, goal_handle):
        self.get_logger().info('Translation')
        t_init = time.time()

        self._init_pose = self.get_current_pose()
        if (self._init_pose is None):
            goal_handle.abort()

        controller = MinimumJerkTrajectoryPlanner()
        pose_target = Pose(goal_handle.request.target.x, 0, 0)
        pose_start = Pose(0, 0, 0)
        dist = abs(pose_target.x)

        max_total = dist/goal_handle.request.speed
        robot = Robot("Robot", "b", max_total, controller,
                      pose_start, pose_target, "r")
        robot.generate_trajectory(dt)
        vel = robot.odometry.velocities

        translation_feedback = Translate.Feedback()
        linear_vel = Twist()
        linear_vel.linear = Vector3()

        xy_tolerance = 0.015
        if (goal_handle.request.xy_goal_tolerance > 0):
            xy_tolerance = goal_handle.request.xy_goal_tolerance

        for vel in robot.odometry.velocities:
            # Ugly way of accepting cancelation, use callbacks later
            if (goal_handle.is_cancel_requested):
                linear_vel.linear.x = 0.0
                self._publisher.publish(linear_vel)
                goal_handle.canceled()

            self._current_pose = self.get_current_pose()
            if (self._current_pose is None):
                goal_handle.abort()

            translation_feedback.distance_traveled = distance(self._current_pose, self._init_pose)
            linear_vel.linear.x = float(vel.x)
            self._publisher.publish(linear_vel)
            goal_handle.publish_feedback(translation_feedback)
            time.sleep(dt)

        linear_vel.linear.x = 0.0
        self._publisher.publish(linear_vel)

        self._current_pose = self.get_current_pose()
        if (self._current_pose is None):
            goal_handle.abort()

        translation_feedback.distance_traveled = distance(self._current_pose, self._init_pose)

        if abs(translation_feedback.distance_traveled - abs(goal_handle.request.target.x)) <= xy_tolerance:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        res = Translate.Result()
        res.total_elapsed_time.sec = int((time.time() - t_init))
        return res
