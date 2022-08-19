#include "minimum_jerk_ros/minimum_jerk_ros.hpp"
#include "minimum_jerk_ros/fill_file.hpp"

namespace minimum_jerk
{
    MinimumJerkRos::MinimumJerkRos(bool intra_process_comms) : rclcpp_lifecycle::LifecycleNode("minimum_jerk_action_server", rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        RCLCPP_INFO(this->get_logger(), "on_initialize()...");
        this->declare_parameter<double>("transform_tolerance", 1.0);
        this->declare_parameter<double>("control_frequency", 30);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_configure()...");

        this->get_parameter("transform_tolerance", this->transform_tolerance_);
        this->get_parameter("control_frequency", this->control_frequency_);

        this->buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        this->action_rotation_server_ = std::make_shared<ActionServerRot>(get_node_base_interface(),
                                                                          get_node_clock_interface(),
                                                                          get_node_logging_interface(),
                                                                          get_node_waitables_interface(),
                                                                          "rotate", std::bind(&MinimumJerkRos::rotation_callback, this));
        this->action_translation_server_ = std::make_shared<ActionServerTrans>(get_node_base_interface(),
                                                                               get_node_clock_interface(),
                                                                               get_node_logging_interface(),
                                                                               get_node_waitables_interface(),
                                                                               "translate", std::bind(&MinimumJerkRos::translation_callback, this));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_activate()...");
        this->action_rotation_server_->activate();
        this->action_translation_server_->activate();
        this->publisher_->on_activate();
        RCLCPP_INFO(this->get_logger(), "Action server successfully activated...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_deactivate()...");
        this->action_rotation_server_->deactivate();
        this->action_translation_server_->deactivate();
        this->publisher_->on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_cleanup()...");
        this->publisher_.reset();
        this->buffer_.reset();
        this->listener_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_shutdown()...");
        this->publisher_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void MinimumJerkRos::rotation_callback()
    {
        auto goal = action_rotation_server_->get_current_goal();
        RCLCPP_INFO(this->get_logger(), "Rotation: %f rad, min_vel: %f, max_vel: %f, acc_lim: %f, collision_check: %i, yaw_goal_tolerance: %f",
                    goal->target_yaw, goal->min_rotational_vel, goal->max_rotational_vel, goal->rotational_acc_lim, goal->enable_collision_check,
                    goal->yaw_goal_tolerance);

        if (goal->target_yaw < -M_PI || goal->target_yaw > M_PI)
        {
            RCLCPP_WARN(this->get_logger(), "Target yaw need to be between -pi and pi");
            action_translation_server_->terminate_current();
            return;
        }
        auto t_init = get_clock()->now();
        try
        {
            this->init_pose_ = this->get_current_pose();
        }
        catch (tf2::TransformException &e)
        {
            action_translation_server_->terminate_current();
            return;
        }
        Pose pose_target = Pose(0, 0, goal->target_yaw);
        Pose pose_start = Pose(0, 0, 0);
        double dist = abs(pose_target.get_theta() - pose_start.get_theta());
        double max_total = dist / goal->min_rotational_vel;
        TrajectoryPlanner controller = TrajectoryPlanner(max_total, 1 / this->control_frequency_);
        Robot robot = Robot("Robot", max_total, controller, pose_start, pose_target, "r");
        robot.generate_trajectory();

        auto rotation_feedback = std::make_shared<Rotation::Feedback>();
        auto angular_vel = geometry_msgs::msg::Twist();
        angular_vel.angular = geometry_msgs::msg::Vector3();
        double yaw_tolerance = 0.1;
        if (goal->yaw_goal_tolerance > 0)
            yaw_tolerance = goal->yaw_goal_tolerance;

        FilFile::fil_file_rotation<Pose>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_poses(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/poses.txt");
        FilFile::fil_file_rotation<Velocity>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_velocities(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/velocities.txt");
        FilFile::fil_file_rotation<Acceleration>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_accelerations(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/accelerations.txt");

        int i = 0;

        int fd_for_times = FilFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/for_times.txt");
        int fd_r_s_times = FilFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/r_s_times.txt");

        auto t_init_for = get_clock()->now();
        for (auto vel : robot.get_odometry().get_velocities())
        {
            if (action_rotation_server_->is_cancel_requested())
            {
                angular_vel.angular.z = 0.0;
                this->publisher_->publish(angular_vel);
                action_rotation_server_->terminate_current();
                return;
            }
            try
            {
                this->current_pose_ = this->get_current_pose();
            }
            catch (tf2::TransformException &e)
            {
                action_translation_server_->terminate_current();
                return;
            }
            rotation_feedback->angular_distance_traveled = abs(this->current_pose_.get_theta() - this->init_pose_.get_theta());
            rotation_feedback->angular_distance_traveled -= 2 * M_PI * std::floor((rotation_feedback->angular_distance_traveled + M_PI) / (2 * M_PI));
            rotation_feedback->angular_distance_traveled = abs(rotation_feedback->angular_distance_traveled);

            angular_vel.angular.z = double(vel.get_theta());
            this->publisher_->publish(angular_vel);

            auto t_begin_for = get_clock()->now();
            auto init_timeout = std::chrono::system_clock::now();
            while (abs(robot.get_odometry().get_poses().at(i).get_theta()) >= rotation_feedback->angular_distance_traveled - yaw_tolerance)
            {
                if (std::chrono::system_clock::now() - init_timeout >= std::chrono::milliseconds(static_cast<int>(1 / this->control_frequency_ * 1000 * 1.15)))
                    break;
                rotation_feedback->angular_distance_traveled = abs(this->current_pose_.get_theta() - this->init_pose_.get_theta());
                rotation_feedback->angular_distance_traveled -= 2 * M_PI * std::floor((rotation_feedback->angular_distance_traveled + M_PI) / (2 * M_PI));
                rotation_feedback->angular_distance_traveled = abs(rotation_feedback->angular_distance_traveled);
            }

            if (abs(rotation_feedback->angular_distance_traveled - abs(goal->target_yaw)) <= yaw_tolerance)
            {
                angular_vel.angular.z = 0.0;
                this->publisher_->publish(angular_vel);
                break;
            }

            action_rotation_server_->publish_feedback(rotation_feedback);

            FilFile::fil_one_ligne(fd_for_times, i, double((get_clock()->now() - t_begin_for).nanoseconds()) * 1e-9);
            FilFile::fil_one_ligne(fd_r_s_times, i, double((get_clock()->now() - t_init_for).nanoseconds()) * 1e-9, robot.get_odometry().get_timestamps().at(i));
            i++;
        }

        angular_vel.angular.z = 0.0;
        this->publisher_->publish(angular_vel);

        FilFile::close_file(fd_for_times);
        FilFile::close_file(fd_r_s_times);

        try
        {
            this->current_pose_ = this->get_current_pose();
        }
        catch (tf2::TransformException &e)
        {
            action_translation_server_->terminate_current();
            return;
        }

        rotation_feedback->angular_distance_traveled = abs(this->current_pose_.get_theta() - this->init_pose_.get_theta());
        rotation_feedback->angular_distance_traveled -= 2 * M_PI * std::floor((rotation_feedback->angular_distance_traveled + M_PI) / (2 * M_PI));
        rotation_feedback->angular_distance_traveled = abs(rotation_feedback->angular_distance_traveled);

        auto res = std::make_shared<Rotation::Result>();
        res->total_elapsed_time.nanosec = int((get_clock()->now() - t_init).nanoseconds());
        RCLCPP_INFO(this->get_logger(), "distance traveled %f, error %f\n", rotation_feedback->angular_distance_traveled, abs(rotation_feedback->angular_distance_traveled - abs(goal->target_yaw)));
        (abs(rotation_feedback->angular_distance_traveled - abs(goal->target_yaw)) <= yaw_tolerance) ? action_rotation_server_->succeeded_current(res) : action_rotation_server_->terminate_current();
    }

    void MinimumJerkRos::translation_callback()
    {
        auto goal = action_translation_server_->get_current_goal();
        RCLCPP_INFO(this->get_logger(), "Translation: %f m, speed: %f, collision_check: %i, xy_goal_tolerance: %f",
                    goal->target.x, goal->speed, goal->enable_collision_check, goal->xy_goal_tolerance);
        auto t_init = get_clock()->now();
        try
        {
            this->init_pose_ = this->get_current_pose();
        }
        catch (tf2::TransformException &e)
        {
            action_translation_server_->terminate_current();
            return;
        }
        Pose pose_target = Pose(goal->target.x, 0, 0);
        Pose pose_start = Pose(0, 0, 0);
        double dist = abs(pose_target.get_x());
        double max_total = dist / goal->speed;
        TrajectoryPlanner controller = TrajectoryPlanner(max_total, 1 / this->control_frequency_);
        Robot robot = Robot("Robot", max_total, controller, pose_start, pose_target, "tx");
        robot.generate_trajectory();

        auto translation_feedback = std::make_shared<Translation::Feedback>();
        auto linear_vel = geometry_msgs::msg::Twist();
        linear_vel.linear = geometry_msgs::msg::Vector3();
        double xy_tolerance = 0.015;
        if (goal->xy_goal_tolerance > 0)
            xy_tolerance = goal->xy_goal_tolerance;

        FilFile::fil_file_translation<Pose>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_poses(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/poses.txt");
        FilFile::fil_file_translation<Velocity>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_velocities(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/velocities.txt");
        FilFile::fil_file_translation<Acceleration>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_accelerations(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/accelerations.txt");

        int i = 0;

        int fd_for_times = FilFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/for_times.txt");
        int fd_r_s_times = FilFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/r_s_times.txt");

        auto t_init_for = get_clock()->now();
        for (auto vel : robot.get_odometry().get_velocities())
        {
            if (action_translation_server_->is_cancel_requested())
            {
                linear_vel.linear.x = 0.0;
                this->publisher_->publish(linear_vel);
                action_translation_server_->terminate_current();
                return;
            }
            try
            {
                this->current_pose_ = this->get_current_pose();
            }
            catch (tf2::TransformException &e)
            {
                action_translation_server_->terminate_current();
                return;
            }
            geometry_msgs::msg::Point start;
            start.x = this->init_pose_.get_x();
            start.y = this->init_pose_.get_y();
            geometry_msgs::msg::Point target;
            target.x = this->current_pose_.get_x();
            target.y = this->current_pose_.get_y();
            translation_feedback->distance_traveled = abs(cmr_geometry_utils::compute_distance_2d(start, target));
            linear_vel.linear.x = double(vel.get_x());
            this->publisher_->publish(linear_vel);

            auto t_begin_for = get_clock()->now();
            auto init_timeout = std::chrono::system_clock::now();

            while (abs(robot.get_odometry().get_poses().at(i).get_x()) >= translation_feedback->distance_traveled - xy_tolerance)
            {
                if (std::chrono::system_clock::now() - init_timeout >= std::chrono::milliseconds(static_cast<int>(1 / this->control_frequency_ * 1000 * 1.15)))
                    break;
                translation_feedback->distance_traveled = abs(cmr_geometry_utils::compute_distance_2d(start, target));
            }

            if (abs(translation_feedback->distance_traveled - abs(goal->target.x)) <= xy_tolerance)
            {
                linear_vel.linear.x = 0.0;
                this->publisher_->publish(linear_vel);
                break;
            }

            action_translation_server_->publish_feedback(translation_feedback);

            FilFile::fil_one_ligne(fd_for_times, i, double((get_clock()->now() - t_begin_for).nanoseconds()) * 1e-9);
            FilFile::fil_one_ligne(fd_r_s_times, i, double((get_clock()->now() - t_init_for).nanoseconds()) * 1e-9, robot.get_odometry().get_timestamps().at(i));
            i++;
        }
        linear_vel.linear.x = 0.0;
        this->publisher_->publish(linear_vel);

        FilFile::close_file(fd_for_times);
        FilFile::close_file(fd_r_s_times);

        try
        {
            this->current_pose_ = this->get_current_pose();
        }
        catch (tf2::TransformException &e)
        {
            action_translation_server_->terminate_current();
            return;
        }
        geometry_msgs::msg::Point start;
        start.x = this->init_pose_.get_x();
        start.y = this->init_pose_.get_y();
        start.z = 0;
        geometry_msgs::msg::Point target;
        target.x = this->current_pose_.get_x();
        target.y = this->current_pose_.get_y();
        target.z = 0;
        translation_feedback->distance_traveled = abs(cmr_geometry_utils::compute_distance_2d(start, target));

        auto res = std::make_shared<Translation::Result>();
        res->total_elapsed_time.nanosec = int((get_clock()->now() - t_init).nanoseconds());
        RCLCPP_INFO(this->get_logger(), "distance traveled %f, error %f\n", translation_feedback->distance_traveled, abs(translation_feedback->distance_traveled - abs(goal->target.x)));
        (abs(translation_feedback->distance_traveled - abs(goal->target.x)) <= xy_tolerance) ? action_translation_server_->succeeded_current(res) : action_rotation_server_->terminate_current();
    }

    minimum_jerk::Pose MinimumJerkRos::get_current_pose()
    {
        Pose pose;
        try
        {
            auto now = get_clock()->now();
            auto trans = buffer_->lookupTransform("odom", "base_footprint", now, rclcpp::Duration::from_seconds(this->transform_tolerance_));
            double theta = tf2::getYaw(trans.transform.rotation);
            theta -= 2 * M_PI * std::floor((theta) / (2 * M_PI));
            pose = Pose(trans.transform.translation.x, trans.transform.translation.y, theta);
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "base_footprint to odom: %s", e.what());
            throw e;
        }
        return pose;
    }
}