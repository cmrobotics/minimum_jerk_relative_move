#pragma once

#include "rclcpp/rclcpp.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include <chrono>
#include <cmr_geometry_utils/basic_geometry_utils.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmr_msgs/action/translate.hpp>
#include <cmr_msgs/action/rotate.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav2_util/simple_action_server.hpp>
#include <functional>
#include <unistd.h>
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"
#include "minimum_jerk_trajectory_planner/robot.hpp"

namespace minimum_jerk
{
    class MinimumJerkRos : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        using Rotation = cmr_msgs::action::Rotate;
        using Translation = cmr_msgs::action::Translate;

        explicit MinimumJerkRos(bool intra_process_comms = false);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

        using ActionServerRot = nav2_util::SimpleActionServer<Rotation>;
        std::shared_ptr<ActionServerRot> action_rotation_server_;
        using ActionServerTrans = nav2_util::SimpleActionServer<Translation>;
        std::shared_ptr<ActionServerTrans> action_translation_server_;

        void rotation_callback();
        void translation_callback();

    private:
        minimum_jerk::Pose init_pose_;
        minimum_jerk::Pose current_pose_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> publisher_;

        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;

        minimum_jerk::Pose get_current_pose();

        double transform_tolerance_;
        double control_frequency_;
    };
}