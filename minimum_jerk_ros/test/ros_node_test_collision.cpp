#include <chrono>
#include <atomic>
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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "minimum_jerk_ros/minimum_jerk_ros.hpp"
#include "minimum_jerk_ros_test/collision_checker_server_mock.hpp"

#include <stdio.h>

static std::atomic<bool> is_action_running;

void collisionHandler(std::shared_ptr<minimum_jerk_test::CollisionCheckerServerMock> collision_node, std::shared_ptr<cmr_tests_utils::SimulatedDifferentialRobot> robot)
{
    while (!is_action_running)
        ;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    collision_node->set_is_collision_free(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    EXPECT_EQ(robot->get_last_velocity_msg()->linear.x, 0);

    collision_node->set_is_collision_free(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    EXPECT_NE(robot->get_last_velocity_msg()->linear.x, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    collision_node->set_is_collision_free(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    EXPECT_EQ(robot->get_last_velocity_msg()->linear.x, 0);

    collision_node->set_is_collision_free(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    EXPECT_NE(robot->get_last_velocity_msg()->linear.x, 0);
}

TEST(RosNodeTestCollision, reach_the_translation_goal_with_collision)
{
    rclcpp::init(0, nullptr);

    auto spinner = cmr_tests_utils::SingleThreadSpinner();

    auto minimum_jerk_node = std::make_shared<minimum_jerk::MinimumJerkRos>("minimum_jerk_controller");
    auto translation_client = std::make_shared<cmr_clients_utils::BasicActionClient<minimum_jerk_msgs::action::Translate>>("translation_client", "translate", 1000);
    auto publisher = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("pub_test_node", "cmd_vel", false);
    auto robot_node = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    auto tf_broadcaster = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("map_to_odom_node", map_to_odom, 30, true);

    auto collision_node = std::make_shared<minimum_jerk_test::CollisionCheckerServerMock>();

    spinner.add_node(collision_node->get_node_base_interface());
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

    minimum_jerk_msgs::action::Translate::Goal goal;
    goal.target_x = 3;
    goal.min_velocity = 0.3;
    goal.enable_collision_check = true;
    goal.xy_goal_tolerance = 0.1;
    goal.enable_data_save = false;

    std::thread t(collisionHandler,collision_node, robot_node);
    is_action_running.store(true);

    auto status = translation_client->send_goal(goal);
    t.join();

    EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

    rclcpp::shutdown();
}
