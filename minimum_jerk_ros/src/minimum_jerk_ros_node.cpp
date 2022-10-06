#include "minimum_jerk_ros/minimum_jerk_ros.hpp"

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;
  auto nh_ = std::make_shared<minimum_jerk::MinimumJerkRos>();

  exe.add_node(nh_->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}