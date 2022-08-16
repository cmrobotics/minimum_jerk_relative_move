#include "minimum_jerk_ros/minimum_jerk_ros.hpp"

namespace minimum_jerk
{
    MinimumJerkRos::MinimumJerkRos(const rclcpp::NodeOptions &options) : Node("minimum_jerk_action_server", options)
    {
        using namespace std::placeholders;
        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        this->action_rotation_server_ = std::make_unique<ActionServerRot>(get_node_base_interface(),
                                                                          get_node_clock_interface(),
                                                                          get_node_logging_interface(),
                                                                          get_node_waitables_interface(),
                                                                          "rotate", std::bind(&MinimumJerkRos::rotation_callback, this));
        this->action_translation_server_ = std::make_unique<ActionServerTrans>(get_node_base_interface(),
                                                                               get_node_clock_interface(),
                                                                               get_node_logging_interface(),
                                                                               get_node_waitables_interface(),
                                                                               "translate", std::bind(&MinimumJerkRos::translation_callback, this));
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }
    void MinimumJerkRos::rotation_callback() {}

    void MinimumJerkRos::translation_callback() {}

}