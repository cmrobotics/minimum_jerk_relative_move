#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>

#include "cmr_clients_utils/basic_service_client.hpp"
#include "cmr_tests_utils/multi_thread_spinner.hpp"
#include "minimum_jerk_ros_test/collision_checker_server_mock.hpp"

#include "cmr_msgs/srv/is_collision_free.hpp"

TEST(CollisionCheckerMock, test_collision_checker_mock)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::MultiThreadSpinner();

  auto collision_node = std::make_shared<minimum_jerk_test::CollisionCheckerServerMock>();
  auto collision_client_node = std::make_shared<cmr_clients_utils::BasicServiceClient<cmr_msgs::srv::IsCollisionFree>>("collision_client_node", "is_collision_free");

  spinner.add_node(collision_node->get_node_base_interface());
  spinner.spin_all_nodes();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Check Initialization
  ASSERT_TRUE(collision_client_node->is_server_ready());
  EXPECT_EQ(collision_node->getLastRequest(), nullptr);

  // Build request
  auto collision_checker_req = std::make_shared<cmr_msgs::srv::IsCollisionFree::Request>();
  collision_checker_req->in_global_costmap = true;

  // Send request
  auto response = collision_client_node->send_request(collision_checker_req);

  // Check request
  EXPECT_TRUE(response);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_collision_free);

  // Build request
  collision_checker_req->in_global_costmap = false;

  // Send request
  response = collision_client_node->send_request(collision_checker_req);

  // Check request
  EXPECT_TRUE(response);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_collision_free);

  // Build request
  collision_checker_req->in_global_costmap = false;
  collision_checker_req->pose.pose.position.x = 11;

  // Send request
  response = collision_client_node->send_request(collision_checker_req);

  // Check request
  EXPECT_TRUE(response);
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->is_collision_free);

  collision_node->set_is_collision_free(true);

  response = collision_client_node->send_request(collision_checker_req);

  // Check request
  EXPECT_TRUE(response);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_collision_free);
  
  rclcpp::shutdown();
}