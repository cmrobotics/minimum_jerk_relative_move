#include "minimum_jerk_relative_move/minimum_jerk_relative_move.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto nh_ = std::make_shared<minimum_jerk_relative_move::MinimumJerkRelativeMove>("minimum_jerk_relative_move");

  exe.add_node(nh_->MinimumJerkRelativeMove());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}