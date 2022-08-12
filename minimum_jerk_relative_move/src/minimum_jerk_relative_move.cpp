#include "minimum_jerk_relative_move/minimum_jerk_relative_move.hpp"

namespace minimum_jerk
{
    MinimumJerkRelativeMove::MinimumJerkRelativeMove(const rclcpp::NodeOptions &options) : Node("minimum_jerk_action_server", options)
    {
        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::qos::qos_profile_system_default);
        this->action_rotation_server_ = this->rclcpp_action::create_server<Rotation>(this, "rotate", std::bind(&MinimumJerkRelativeMove::rotation_callback, this, _1));
        this->action_translation_server_ = this->rclcpp_action::create_server<Translation>(this, "translate", std::bind(&MinimumJerkRelativeMove::translation_callback, this, _1));
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }
    void MinimumJerkRelativeMove::rotation_callback(const std::shared_ptr<GoalHandleRotation> goal_handle){}

    void MinimumJerkRelativeMove::translation_callback(const std::shared_ptr<GoalHandleTranslation> goal_handle){}


}