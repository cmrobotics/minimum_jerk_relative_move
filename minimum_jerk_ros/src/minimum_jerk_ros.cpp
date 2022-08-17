#include "minimum_jerk_ros/minimum_jerk_ros.hpp"

#define dt 0.1

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
    void MinimumJerkRos::rotation_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Rotation");
        auto goal = action_rotation_server_->get_current_goal();
        auto t_init = get_clock()->now();
        this->init_pose_ = this->get_current_pose();
        if (this->init_pose_.get_x() < 0)
            action_rotation_server_->terminate_current();
        Pose pose_target = Pose(0, 0, goal->target_yaw);
        Pose pose_start = Pose(0, 0, 0);
        double dist = abs(pose_target.get_theta() - pose_start.get_theta());
        double max_total = dist / goal->min_rotational_vel;
        TrajectoryPlanner controller = TrajectoryPlanner(max_total, dt);
        Robot robot = Robot("Robot", max_total, controller, pose_start, pose_target, "r");
        robot.generate_trajectory();

        auto rotation_feedback = std::make_shared<Rotation::Feedback>();
        auto angular_vel = geometry_msgs::msg::Twist();
        angular_vel.angular = geometry_msgs::msg::Vector3();
        double yaw_tolerance = 0.1;
        if (goal->yaw_goal_tolerance > 0)
            yaw_tolerance = goal->yaw_goal_tolerance;
        for (auto vel : robot.get_odometry().get_velocities())
        {
            if (action_rotation_server_->is_cancel_requested())
            {
                angular_vel.angular.z = 0.0;
                this->publisher_->publish(angular_vel);
                action_rotation_server_->terminate_current();
            }
            this->current_pose_ = this->get_current_pose();
            if (this->current_pose_.get_x() < 0)
                action_rotation_server_->terminate_current();
            rotation_feedback->angular_distance_traveled = abs(
                this->current_pose_.get_theta() - this->init_pose_.get_theta());
            angular_vel.angular.z = double(vel.get_theta());
            this->publisher_->publish(angular_vel);
            action_rotation_server_->publish_feedback(rotation_feedback);
            sleep(dt);
        }
        angular_vel.angular.z = 0.0;
        this->publisher_->publish(angular_vel);
        action_rotation_server_->terminate_current();
        if (this->current_pose_.get_x() < 0)
            action_rotation_server_->terminate_current();
        rotation_feedback->angular_distance_traveled = abs(
            this->current_pose_.get_theta() - this->init_pose_.get_theta());

        auto res = std::make_shared<Rotation::Result>();
        res->total_elapsed_time.nanosec = int((get_clock()->now() - t_init).nanoseconds());
        (abs(rotation_feedback->angular_distance_traveled - abs(goal->target_yaw)) <= yaw_tolerance)? action_rotation_server_->succeeded_current(res):action_rotation_server_->terminate_current();
    }

    void MinimumJerkRos::translation_callback() {}

    minimum_jerk::Pose MinimumJerkRos::get_current_pose()
    {
        Pose pose;
        try
        {
            auto now = get_clock()->now();
            auto trans = buffer_->lookupTransform("odom", "base_footprint", now);
            double theta = tf2::getYaw(trans.transform.rotation);
            pose = Pose(trans.transform.translation.x, trans.transform.translation.y, theta);
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "base_footprint to odom: %s", e.what());
            return Pose(-1, -1, -1);
        }
        return pose;
    }

}