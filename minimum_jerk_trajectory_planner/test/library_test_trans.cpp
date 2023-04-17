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

#include "minimum_jerk_trajectory_planner/robot.hpp"
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include "minimum_jerk_trajectory_planner/trajectory.hpp"
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"

#include <stdio.h>

TEST(LibraryTestTrans, reach_the_negative_translation_goal)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
  auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  spinner.add_node(robot_node->get_node_base_interface());
  spinner.add_node(publisher->get_node_base_interface());
  spinner.spin_some_all_nodes();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int k = 0; k < 3; k++) {
    robot_node->reset_odometry();

    geometry_msgs::msg::Twist linear_vel;
    linear_vel.linear = geometry_msgs::msg::Vector3();
    double xy_tolerance = 0.1;

    double target = -1.5 + k * 0.5;
    minimum_jerk::Pose pose_target = minimum_jerk::Pose(target, 0, 0);
    minimum_jerk::Pose pose_start = minimum_jerk::Pose(0, 0, 0);
    double dist = abs(pose_target.get_x() - pose_start.get_x());
    double max_total = dist / 0.1;
    minimum_jerk::TrajectoryPlanner controller = minimum_jerk::TrajectoryPlanner(max_total, 0.3);
    minimum_jerk::Robot robot = minimum_jerk::Robot("Robot", max_total, controller, pose_start, pose_target, "tx");
    robot.generate_trajectory();

    double robot_x;

    auto odometry = robot.get_odometry();
    int i = 0;
    for (auto vel : odometry.get_velocities()) {
      linear_vel.linear.x = static_cast<double>(vel.get_x());
      publisher->publish(linear_vel);

      auto init_timeout = std::chrono::system_clock::now();
      robot_x = abs(robot_node->get_transform().transform.translation.x);

      while (abs(odometry.get_poses().at(i).get_x()) >= robot_x - xy_tolerance) {
        if (std::chrono::system_clock::now() - init_timeout >= std::chrono::milliseconds(static_cast<int>(0.3 * 1000 * 1.15)))
          break;
        robot_x = abs(robot_node->get_transform().transform.translation.x);
      }

      if (abs(robot_x - abs(target)) <= xy_tolerance) {
        linear_vel.linear.x = 0;

        publisher->publish(linear_vel);
        break;
      }
      i++;
    }

    linear_vel.linear.x = 0;
    publisher->publish(linear_vel);

    robot_x = abs(robot_node->get_transform().transform.translation.x);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Precision: %f", abs(robot_x - abs(target)));
    EXPECT_EQ(abs(robot_x - abs(target)) <= xy_tolerance, true);
  }
  rclcpp::shutdown();
}

TEST(LibraryTestTrans, reach_the_positive_translation_goal)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
  auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  spinner.add_node(robot_node->get_node_base_interface());
  spinner.add_node(publisher->get_node_base_interface());
  spinner.spin_some_all_nodes();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int k = 0; k < 3; k++) {
    robot_node->reset_odometry();

    geometry_msgs::msg::Twist linear_vel;
    linear_vel.linear = geometry_msgs::msg::Vector3();
    double xy_tolerance = 0.1;

    double target = 0.5 + k * 0.5;
    minimum_jerk::Pose pose_target = minimum_jerk::Pose(target, 0, 0);
    minimum_jerk::Pose pose_start = minimum_jerk::Pose(0, 0, 0);
    double dist = abs(pose_target.get_x() - pose_start.get_x());
    double max_total = dist / 0.1;
    minimum_jerk::TrajectoryPlanner controller = minimum_jerk::TrajectoryPlanner(max_total, 0.3);
    minimum_jerk::Robot robot = minimum_jerk::Robot("Robot", max_total, controller, pose_start, pose_target, "tx");
    robot.generate_trajectory();

    double robot_x;

    auto odometry = robot.get_odometry();
    int i = 0;
    for (auto vel : odometry.get_velocities()) {
      linear_vel.linear.x = static_cast<double>(vel.get_x());
      publisher->publish(linear_vel);

      auto init_timeout = std::chrono::system_clock::now();
      robot_x = abs(robot_node->get_transform().transform.translation.x);

      while (abs(odometry.get_poses().at(i).get_x()) >= robot_x - xy_tolerance) {
        if (std::chrono::system_clock::now() - init_timeout >= std::chrono::milliseconds(static_cast<int>(0.3 * 1000 * 1.15)))
          break;
        robot_x = abs(robot_node->get_transform().transform.translation.x);
      }

      if (abs(robot_x - abs(target)) <= xy_tolerance) {
        linear_vel.linear.x = 0;

        publisher->publish(linear_vel);
        break;
      }
      i++;
    }

    linear_vel.linear.x = 0;
    publisher->publish(linear_vel);

    robot_x = abs(robot_node->get_transform().transform.translation.x);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Precision: %f", abs(robot_x - abs(target)));
    EXPECT_EQ(abs(robot_x - abs(target)) <= xy_tolerance, true);
  }
  rclcpp::shutdown();
}

TEST(LibraryTestTrans, reach_the_zero_translation_goal)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
  auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  spinner.add_node(robot_node->get_node_base_interface());
  spinner.add_node(publisher->get_node_base_interface());
  spinner.spin_some_all_nodes();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  robot_node->reset_odometry();

  geometry_msgs::msg::Twist linear_vel;
  linear_vel.linear = geometry_msgs::msg::Vector3();

  double target = 0;
  minimum_jerk::Pose pose_target = minimum_jerk::Pose(0, 0, target);
  minimum_jerk::Pose pose_start = minimum_jerk::Pose(0, 0, 0);
  double dist = abs(pose_target.get_x() - pose_start.get_x());
  double max_total = dist / 0.1;
  minimum_jerk::TrajectoryPlanner controller = minimum_jerk::TrajectoryPlanner(max_total, 0.3);
  minimum_jerk::Robot robot = minimum_jerk::Robot("Robot", max_total, controller, pose_start, pose_target, "tx");

  robot.generate_trajectory();

  EXPECT_EQ(controller.get_list_poses().empty(), true);
  rclcpp::shutdown();
}
