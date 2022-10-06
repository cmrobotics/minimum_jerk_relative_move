#pragma once

#include "rclcpp/rclcpp.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include <chrono>
#include <cmr_geometry_utils/basic_geometry_utils.hpp>
#include <cmr_clients_utils/basic_service_client.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <minimum_jerk_msgs/action/translate.hpp>
#include <minimum_jerk_msgs/action/rotate.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <cmr_msgs/srv/is_collision_free.hpp>
#include <cmr_geometry_utils/transforms_utils.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <functional>
#include <unistd.h>
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"
#include "minimum_jerk_trajectory_planner/robot.hpp"

namespace minimum_jerk
{
class MinimumJerkRos : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Rotation = minimum_jerk_msgs::action::Rotate;
  using Translation = minimum_jerk_msgs::action::Translate;

  explicit MinimumJerkRos(bool intra_process_comms = false);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  using ActionServerRot = nav2_util::SimpleActionServer<Rotation>;
  std::shared_ptr<ActionServerRot> action_rotation_server_;
  using ActionServerTrans = nav2_util::SimpleActionServer<Translation>;
  std::shared_ptr<ActionServerTrans> action_translation_server_;

  void rotation_callback();
  void translation_callback();

private:
  minimum_jerk::Pose init_pose_;
  minimum_jerk::Pose current_pose_;
  double idle_timeout_;
  double obstacle_lookahead_distance_;
  double scaled_obstacle_lookahead_distance_;
  double robot_radius_;
  double transform_tolerance_;
  double control_frequency_;
  bool debug_trajectory_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> publisher_;

  std::shared_ptr<cmr_clients_utils::BasicServiceClient<cmr_msgs::srv::IsCollisionFree>> collision_srv_client_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> trajectory_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> goal_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> obstacles_marker_pub_; //!< Publisher for visualization markers

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  minimum_jerk::Pose get_current_pose_();
  bool is_obstacle_on_path_(std::vector<geometry_msgs::msg::PoseStamped> path);
  bool is_collision_free_(const geometry_msgs::msg::PoseStamped &pose);
  std::size_t get_rank_of_closest_pose_in_path_(const geometry_msgs::msg::PoseStamped &pose, std::vector<geometry_msgs::msg::PoseStamped> path);
  std::vector<geometry_msgs::msg::PoseStamped> convert_poses_to_posestampeds_(std::vector<minimum_jerk::Pose> poses);
  geometry_msgs::msg::Quaternion to_quaternion_(double yaw);
  bool show_trajectory_(std::vector<geometry_msgs::msg::PoseStamped> poses);
  void stop_robot_(geometry_msgs::msg::Twist linear_vel);


  Trajectory compute_translation_velocities_(const std::shared_ptr<const minimum_jerk_msgs::action::Translate::Goal> &goal);
  bool check_translation_collision_(std::vector<geometry_msgs::msg::PoseStamped> poses, geometry_msgs::msg::Twist linear_vel, double* t_collision);
  double compute_distance_();

  Trajectory compute_rotation_velocities_(const std::shared_ptr<const minimum_jerk_msgs::action::Rotate::Goal> &goal);
  bool check_rotation_collision_(std::vector<geometry_msgs::msg::PoseStamped> poses, geometry_msgs::msg::Twist angular_vel, double* t_collision);
  double compute_angle_();
};
}