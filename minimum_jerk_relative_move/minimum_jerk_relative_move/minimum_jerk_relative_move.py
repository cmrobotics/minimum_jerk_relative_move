import rclpy
import time
from rclpy.node import Node
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
from .mathematical_fonctions import euler_from_quaternion

dt = 0.1


class MinimumJerkRelativeMove(Node):

    def __init__(self):

        self._init_pose = {"x": 0, "angle": 0}
        self._current_pose = {"x": 0, "angle": 0}

        super().__init__('minimal_jerk_relative_move')
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._action_rotation_server = ActionServer(
            self, Rotate, 'rotate', self.rotation_callback)
        self._action_translation_server = ActionServer(
            self, Translate, 'translate', self.translation_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def set_current_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_footprint to odom: {ex}')
            return
        self._current_pose["x"] = trans.transform.translation.x
        self._current_pose["angle"] = euler_from_quaternion(
            trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)["z"]

    def rotation_callback(self, goal_handle):
        self.get_logger().info('Rotation')

        t_init = time.time()


        self.set_current_pose()
        self._init_pose = self._current_pose
        controller = MinimumJerkTrajectoryPlanner()
        pose_target = Pose(0, 0, goal_handle.request.target_yaw)
        pose_start = Pose(0, 0, self._init_pose["angle"])
        dist = abs(pose_target.theta-pose_start.theta)
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
            self.set_current_pose()
            rotation_feedback.angular_distance_traveled = self._current_pose[
                "angle"] - self._init_pose["angle"]
            angular_vel.angular.z = float(vel.theta)
            self._publisher.publish(angular_vel)
            goal_handle.publish_feedback(rotation_feedback)
            time.sleep(dt)

        angular_vel.angular.z = 0.0
        self._publisher.publish(angular_vel)
        self.set_current_pose()
        if (abs(self._current_pose["angle"] - (self._init_pose["angle"] +goal_handle.request.target_yaw)) < yaw_tolerance):
            goal_handle.succeed()
        res = Rotate.Result()
        res.total_elapsed_time.sec = int((time.time() - t_init))
        return res

    def translation_callback(self, goal_handle):
        self.get_logger().info('Translation')
        t_init = time.time()

        self.set_current_pose()
        self._init_pose = self._current_pose

        controller = MinimumJerkTrajectoryPlanner()
        pose_target = Pose(goal_handle.request.target.x, 0, 0)
        pose_start = Pose(self._init_pose["x"], 0, 0)
        dist = abs(pose_target.x-pose_start.x)
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
            self.set_current_pose()
            translation_feedback.distance_traveled = self._current_pose["x"] - self._init_pose["x"]
            linear_vel.linear.x = float(vel.x)
            self._publisher.publish(linear_vel)
            goal_handle.publish_feedback(translation_feedback)
            time.sleep(dt)

        linear_vel.linear.x = 0.0
        self._publisher.publish(linear_vel)
        self.set_current_pose()
        if (abs(self._current_pose["x"] - (self._init_pose["x"] + goal_handle.request.target.x)) < xy_tolerance):
            goal_handle.succeed()
        res = Translate.Result()
        res.total_elapsed_time.sec = int((time.time() - t_init))
        return res
