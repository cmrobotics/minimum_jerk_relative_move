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
#include <nav2_util/simple_action_server.hpp>
#include <functional>
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"
#include "minimum_jerk_trajectory_planner/robot.hpp"

namespace minimum_jerk
{
    class MinimumJerkRos : public rclcpp::Node
    {
    public:
        using Rotation = cmr_msgs::action::Rotate;
        using GoalHandleRotation = rclcpp_action::ServerGoalHandle<Rotation>;
        using Translation = cmr_msgs::action::Translate;
        using GoalHandleTranslation = rclcpp_action::ServerGoalHandle<Translation>;
        explicit MinimumJerkRos(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        minimum_jerk::Pose init_pose;
        minimum_jerk::Pose current_pose;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> publisher_;
        using ActionServerRot = nav2_util::SimpleActionServer<Rotation>;
        std::shared_ptr<ActionServerRot> action_rotation_server_;
        using ActionServerTrans = nav2_util::SimpleActionServer<Translation>;
        std::shared_ptr<ActionServerTrans> action_translation_server_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;

        minimum_jerk::Pose get_current_pose();
        void rotation_callback();
        void translation_callback();
    };
}