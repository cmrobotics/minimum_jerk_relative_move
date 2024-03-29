#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <cmr_clients_utils/basic_action_client.hpp>
#include <cmr_tests_utils/multi_thread_spinner.hpp>
#include <cmr_tests_utils/simulated_differential_robot.hpp>
#include <cmr_tests_utils/basic_tf_broadcaster_node_test.hpp>
#include <cmr_tests_utils/basic_publisher_node_test.hpp>
#include <cmr_tests_utils/basic_tf_listener_node_test.hpp>
#include <cmr_tests_utils/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "minimum_jerk_ros/minimum_jerk_ros.hpp"

#include <stdio.h>

TEST(RosNodeTestRot, reach_the_negative_rotation_goal)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  auto minimum_jerk_node = std::make_shared<minimum_jerk::MinimumJerkRos>("minimum_jerk_controller");
  auto rotation_client = std::make_shared<cmr_clients_utils::BasicActionClient<minimum_jerk_msgs::action::Rotate>>("rotation_client", "rotate", 1000);
  auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
  auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  auto tf_broadcaster = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("map_to_odom_node", map_to_odom, 30, true);

  spinner.add_node(robot_node->get_node_base_interface());
  spinner.add_node(minimum_jerk_node->get_node_base_interface());
  spinner.add_node(publisher->get_node_base_interface());
  spinner.add_node(tf_broadcaster->get_node_base_interface());
  spinner.spin_some_all_nodes();

  rclcpp::Parameter use_sim_time("use_sim_time", true);
  minimum_jerk_node->set_parameter(use_sim_time);

  minimum_jerk_node->configure();
  minimum_jerk_node->activate();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int k = 0; k < 3; k++) {
    robot_node->reset_odometry();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    minimum_jerk_msgs::action::Rotate::Goal goal;
    goal.target_yaw = -1.5 + k * 0.5;
    goal.min_velocity = 0.3;
    goal.enable_collision_check = false;
    goal.yaw_goal_tolerance = 0.1;
    goal.enable_data_save = false;

    auto status = rotation_client->send_goal(goal);

    EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);
  }
  rclcpp::shutdown();
}

TEST(RosNodeTestRot, reach_the_positive_rotation_goal)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  auto minimum_jerk_node = std::make_shared<minimum_jerk::MinimumJerkRos>("minimum_jerk_controller");
  auto rotation_client = std::make_shared<cmr_clients_utils::BasicActionClient<minimum_jerk_msgs::action::Rotate>>("rotation_client", "rotate", 1000);
  auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
  auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  auto tf_broadcaster = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("map_to_odom_node", map_to_odom, 30, true);

  spinner.add_node(robot_node->get_node_base_interface());
  spinner.add_node(minimum_jerk_node->get_node_base_interface());
  spinner.add_node(publisher->get_node_base_interface());
  spinner.add_node(tf_broadcaster->get_node_base_interface());
  spinner.spin_some_all_nodes();

  rclcpp::Parameter use_sim_time("use_sim_time", true);
  minimum_jerk_node->set_parameter(use_sim_time);

  minimum_jerk_node->configure();
  minimum_jerk_node->activate();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Rotate 3.0 degrees
  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  minimum_jerk_msgs::action::Rotate::Goal goal;
  goal.target_yaw = 3.14 / 30;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.05;
  goal.enable_data_save = false;
  auto status = rotation_client->send_goal(goal);
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  // Rotate 9.0
  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  goal.target_yaw = 3.14 / 10;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.05;
  goal.enable_data_save = false;
  status = rotation_client->send_goal(goal);
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  // Rotate 15
  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  goal.target_yaw = 3.14 / 6;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.05;
  goal.enable_data_save = false;
  status = rotation_client->send_goal(goal);
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  // Rotate 30
  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  goal.target_yaw = 3.14 / 3;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.05;
  goal.enable_data_save = false;
  status = rotation_client->send_goal(goal);
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  // Rotate 45
  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  goal.target_yaw = 3.14 / 2;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.05;
  goal.enable_data_save = false;
  status = rotation_client->send_goal(goal);
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  // Rotate 90
  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  goal.target_yaw = 3.14;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.05;
  goal.enable_data_save = false;
  status = rotation_client->send_goal(goal);
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  rclcpp::shutdown();
}

TEST(RosNodeTestRot, reach_the_zero_rotation_goal)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  auto minimum_jerk_node = std::make_shared<minimum_jerk::MinimumJerkRos>("minimum_jerk_controller");
  auto rotation_client = std::make_shared<cmr_clients_utils::BasicActionClient<minimum_jerk_msgs::action::Rotate>>("rotation_client", "rotate", 1000);
  auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
  auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  auto tf_broadcaster = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("map_to_odom_node", map_to_odom, 30, true);

  spinner.add_node(robot_node->get_node_base_interface());
  spinner.add_node(minimum_jerk_node->get_node_base_interface());
  spinner.add_node(publisher->get_node_base_interface());
  spinner.add_node(tf_broadcaster->get_node_base_interface());
  spinner.spin_some_all_nodes();

  rclcpp::Parameter use_sim_time("use_sim_time", true);
  minimum_jerk_node->set_parameter(use_sim_time);

  minimum_jerk_node->configure();
  minimum_jerk_node->activate();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  robot_node->reset_odometry();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  minimum_jerk_msgs::action::Rotate::Goal goal;
  goal.target_yaw = 0.0;
  goal.min_velocity = 0.3;
  goal.enable_collision_check = false;
  goal.yaw_goal_tolerance = 0.1;
  goal.enable_data_save = false;

  auto status = rotation_client->send_goal(goal);

  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);
  rclcpp::shutdown();
}
