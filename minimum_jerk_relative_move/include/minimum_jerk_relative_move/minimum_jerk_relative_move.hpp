#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmr_msgs/action/translate.hpp>
#include <cmr_msgs/action/rotate.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
// #include <minimum_jerk_trajectory_planner/trajectory_planners.hpp>
// #include <minimum_jerk_trajectory_planner/pose.hpp>
// #include <minimum_jerk_trajectory_planner/robot.hpp>

namespace minimum_jerk
{
    class MinimumJerkRelativeMove : public rclcpp::Node
    {
    public:
        using Rotation = cmr_msgs::action::Rotate;
        using GoalHandleRotation = rclcpp_action::ServerGoalHandle<Rotation>;
        using Translation = cmr_msgs::action::Translate;
        using GoalHandleTranslation = rclcpp_action::ServerGoalHandle<Translation>;
        explicit MinimumJerkRelativeMove(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        Pose init_pose;
        Pose current_pose;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> publisher_;
        std::shared_ptr<rclcpp_action::Server<Rotation>> action_rotation_server_;
        std::shared_ptr<rclcpp_action::Server<Translation>> action_translation_server_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;

        Pose get_current_pose();
        void rotation_callback(const std::shared_ptr<GoalHandleRotation> goal_handle);
        void translation_callback(const std::shared_ptr<GoalHandleTranslation> goal_handle);
    };
}