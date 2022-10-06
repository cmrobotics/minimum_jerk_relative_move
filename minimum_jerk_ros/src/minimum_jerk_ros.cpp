#include "minimum_jerk_ros/minimum_jerk_ros.hpp"
#include "minimum_jerk_ros/fill_file.hpp"

using namespace std::chrono_literals;

namespace minimum_jerk
{
    MinimumJerkRos::MinimumJerkRos(bool intra_process_comms) : rclcpp_lifecycle::LifecycleNode("minimum_jerk_ros", rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
        RCLCPP_INFO(this->get_logger(), "on_initialize()...");
        this->declare_parameter<double>("transform_tolerance", 1.0);
        this->declare_parameter<double>("control_frequency", 30.0);
        this->declare_parameter<double>("idle_timeout", 10.0);
        this->declare_parameter<double>("obstacle_lookahead_distance", 1);
        this->declare_parameter<double>("robot_radius", 0.26);
        this->declare_parameter<bool>("debug_trajectory", true);
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_configure()...");

        this->get_parameter("robot_radius", robot_radius_);
        this->get_parameter("obstacle_lookahead_distance", this->obstacle_lookahead_distance_);
        this->get_parameter("control_frequency", this->control_frequency_);
        this->get_parameter("idle_timeout", this->idle_timeout_);
        this->get_parameter("debug_trajectory", this->debug_trajectory_);

        this->get_parameter("transform_tolerance", this->transform_tolerance_);

        this->init_pose_ = Pose();
        this->current_pose_ = Pose();

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

        this->collision_srv_client_ = std::make_shared<cmr_clients_utils::BasicServiceClient<cmr_msgs::srv::IsCollisionFree>>("service_client_collision", "is_collision_free");

        obstacles_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("minimum_jerk_obstacle_markers", 1);
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("minimum_jerk_trajectory", 1);
        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("minimum_jerk_goal_pose", 1);

        obstacle_lookahead_distance_ = std::clamp(obstacle_lookahead_distance_, robot_radius_ + 0.1, 1.0);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_activate()...");
        this->action_rotation_server_->activate();
        this->action_translation_server_->activate();
        this->publisher_->on_activate();
        this->obstacles_marker_pub_->on_activate();
        this->goal_pose_pub_->on_activate();
        this->trajectory_pub_->on_activate();
        RCLCPP_INFO(this->get_logger(), "Action server successfully activated...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_deactivate()...");
        this->action_rotation_server_->deactivate();
        this->action_translation_server_->deactivate();
        this->publisher_->on_deactivate();
        this->obstacles_marker_pub_->on_deactivate();
        this->goal_pose_pub_->on_deactivate();
        this->trajectory_pub_->on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_cleanup()...");
        this->publisher_.reset();
        this->buffer_.reset();
        this->listener_.reset();
        this->collision_srv_client_.reset();
        this->obstacles_marker_pub_.reset();
        this->goal_pose_pub_.reset();
        this->trajectory_pub_.reset();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MinimumJerkRos::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_shutdown()...");
        this->publisher_.reset();
        this->goal_pose_pub_.reset();
        this->trajectory_pub_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void MinimumJerkRos::rotation_callback()
    {
        if (action_translation_server_->is_running())
        {
            RCLCPP_ERROR(get_logger(), "Requested Rotation while Translation is running. That's forbidden.");
            action_rotation_server_->terminate_current();
        }

        auto goal = action_rotation_server_->get_current_goal();
        RCLCPP_INFO(this->get_logger(), "Rotation: %f rad, min_vel: %f, collision_check: %i, yaw_goal_tolerance: %f, data_save: %i",
                    goal->target_yaw, goal->min_velocity, goal->enable_collision_check, goal->yaw_goal_tolerance, goal->enable_data_save);

        if (goal->target_yaw <= goal->yaw_goal_tolerance)
        {
            auto r = std::make_shared<Rotation::Result>();
            r->total_elapsed_time.nanosec = 0;
            action_rotation_server_->succeeded_current(r);
            return;
        }

        auto t_init = get_clock()->now();

        try {
            this->init_pose_ = this->get_current_pose_();
        } catch (tf2::TransformException &e) {
            action_rotation_server_->terminate_current();
            return;
        }

        double target = goal->target_yaw;

        if (goal->target_yaw < -M_PI || goal->target_yaw > M_PI)
        {
            target -= 2 * M_PI * std::floor((target + M_PI) / (2 * M_PI));
        }

        auto rotation_feedback = std::make_shared<Rotation::Feedback>();
        auto angular_vel = geometry_msgs::msg::Twist();
        angular_vel.angular = geometry_msgs::msg::Vector3();
        double yaw_tolerance;
        (goal->yaw_goal_tolerance > 0) ? yaw_tolerance = goal->yaw_goal_tolerance : yaw_tolerance = 0.1;

        auto odometry = compute_rotation_velocities_(goal);
        auto poses = convert_poses_to_posestampeds_(odometry.get_poses());

        if (poses.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Error transformation of the path");
            action_rotation_server_->terminate_all();
            return;
        }

        if (debug_trajectory_)
        {
            show_trajectory_(poses);
        }

        int i = 0;
        int fd_for_times;
        int fd_r_s_times;

        if (goal->enable_data_save)
        {
            fd_for_times = FillFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/for_times.txt");
            fd_r_s_times = FillFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/r_s_times.txt");
        }

        auto t_init_for = get_clock()->now();

        for (auto vel : odometry.get_velocities())
        {
            double t_collision = 0;

            if (action_rotation_server_->is_cancel_requested())
            {
                stop_robot_(angular_vel);
                RCLCPP_ERROR(get_logger(), "Rotation stopped: action cancelled by client");
                action_rotation_server_->terminate_current();
                return;
            }

            try {
                this->current_pose_ = this->get_current_pose_();
            } catch (tf2::TransformException &e) {
                RCLCPP_ERROR(get_logger(), "Rotation failed: %s", e.what());
                action_rotation_server_->terminate_current();
                return;
            }
            rotation_feedback->angular_distance_traveled = compute_angle_();

            if (goal->enable_collision_check)
            {
                if (!check_rotation_collision_(poses, angular_vel, &t_collision))
                    return;
            }

            angular_vel.angular.z = static_cast<double>(vel.get_theta());
            this->publisher_->publish(angular_vel);
            scaled_obstacle_lookahead_distance_ = obstacle_lookahead_distance_ + obstacle_lookahead_distance_ * angular_vel.angular.z;
            auto t_begin_for = get_clock()->now();
            auto init_timeout = std::chrono::system_clock::now();

            while (abs(odometry.get_poses().at(i).get_theta()) >= rotation_feedback->angular_distance_traveled - yaw_tolerance)
            {
                if (std::chrono::system_clock::now() - init_timeout >= std::chrono::milliseconds(static_cast<int>(t_collision + 1 / this->control_frequency_ * 1000 * 1.15))) break;

                rotation_feedback->angular_distance_traveled = compute_angle_();

                if (goal->enable_collision_check) if (!check_rotation_collision_(poses, angular_vel, &t_collision)) return;
            }

            if (abs(rotation_feedback->angular_distance_traveled - abs(target)) <= yaw_tolerance)
            {
                stop_robot_(angular_vel);
                break;
            }

            action_rotation_server_->publish_feedback(rotation_feedback);

            if (goal->enable_data_save)
            {
                FillFile::fill_one_ligne(fd_for_times, i, static_cast<double>((get_clock()->now() - t_begin_for).nanoseconds()) * 1e-9);
                FillFile::fill_one_ligne(fd_r_s_times, i, static_cast<double>((get_clock()->now() - t_init_for).nanoseconds()) * 1e-9, odometry.get_timestamps().at(i));
            }
            
            i++;
        }

        stop_robot_(angular_vel);

        if (goal->enable_data_save)
        {
            FillFile::close_file(fd_for_times);
            FillFile::close_file(fd_r_s_times);
        }

        try {
            this->current_pose_ = this->get_current_pose_();
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(get_logger(), "Rotation failed: %s", e.what());
            action_rotation_server_->terminate_current();
            return;
        }

        rotation_feedback->angular_distance_traveled = compute_angle_();
        auto res = std::make_shared<Rotation::Result>();
        res->total_elapsed_time.nanosec = int((get_clock()->now() - t_init).nanoseconds());
        auto error = abs(rotation_feedback->angular_distance_traveled - abs(target));

        RCLCPP_INFO(get_logger(), "Expected Movement: %.4f\nDistance Traveled: %.4f\n Tolerance: %.4f\n Error: %.4f", 
                goal->target_yaw, rotation_feedback->angular_distance_traveled, yaw_tolerance, error
        )
        if (error <= yaw_tolerance) {
            RCLCPP_INFO(get_logger(), "Rotation was a success!");
            action_rotation_server_->succeeded_current(res)
        }  else {
            RCLCPP_ERROR(get_logger(), "Rotation failed!");
            action_rotation_server_->terminate_current();
        } 
    }

    void MinimumJerkRos::translation_callback()
    {
        
        if (action_rotation_server_->is_running())
        {
            RCLCPP_ERROR(get_logger(), "Requested Translation while Rotation is running. That's forbidden.");
            action_translation_server_->terminate_current();
        }

        auto goal = action_translation_server_->get_current_goal();
        RCLCPP_INFO(this->get_logger(), "Translation: %f m, min_velocity: %f, collision_check: %i, xy_goal_tolerance: %f, data_save: %i",
                    goal->target_x, goal->min_velocity, goal->enable_collision_check, goal->xy_goal_tolerance, goal->enable_data_save);
        
        if (goal->target_x == 0)
        {
            auto r = std::make_shared<Translation::Result>();
            r->total_elapsed_time.nanosec = 0;
            action_translation_server_->succeeded_current(r);
            return;
        }
        
        auto t_init = get_clock()->now();
        try {
            this->init_pose_ = this->get_current_pose_();
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(get_logger(), "Translation failed: %s", e.what());
            action_translation_server_->terminate_current();
            return;
        }

        auto translation_feedback = std::make_shared<Translation::Feedback>();
        auto linear_vel = geometry_msgs::msg::Twist();
        linear_vel.linear = geometry_msgs::msg::Vector3();
        double xy_tolerance;
        (goal->xy_goal_tolerance > 0) ? xy_tolerance = goal->xy_goal_tolerance : xy_tolerance = 0.015;
        auto odometry = compute_translation_velocities_(goal);
        auto poses = convert_poses_to_posestampeds_(odometry.get_poses());

        if (poses.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Error transformation of the path");
            action_translation_server_->terminate_all();
            return;
        }

        if (debug_trajectory_)
        {
            show_trajectory_(poses);
        }

        int i = 0;
        int fd_for_times;
        int fd_r_s_times;

        if (goal->enable_data_save)
        {
            fd_for_times = FillFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/for_times.txt");
            fd_r_s_times = FillFile::open_file("./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/r_s_times.txt");
        }

        auto t_init_for = get_clock()->now();

        for (auto vel : odometry.get_velocities())
        {
            double t_collision = 0;

            if (action_translation_server_->is_cancel_requested())
            {
                stop_robot_(linear_vel);
                action_translation_server_->terminate_current();
                return;
            }

            try {
                this->current_pose_ = this->get_current_pose_();
            } catch (tf2::TransformException &e) {
                action_translation_server_->terminate_current();
                return;
            }

            translation_feedback->distance_traveled = compute_distance_();
            if (goal->enable_collision_check)
            {
                if (!check_translation_collision_(poses, linear_vel, &t_collision))
                    return;
            }

            linear_vel.linear.x = static_cast<double>(vel.get_x());
            this->publisher_->publish(linear_vel);
            scaled_obstacle_lookahead_distance_ = obstacle_lookahead_distance_ + obstacle_lookahead_distance_ * linear_vel.linear.x;
            auto t_begin_for = get_clock()->now();
            auto init_timeout = std::chrono::system_clock::now();

            while (abs(odometry.get_poses().at(i).get_x()) >= translation_feedback->distance_traveled - xy_tolerance)
            {
                if (std::chrono::system_clock::now() - init_timeout >= std::chrono::milliseconds(static_cast<int>(t_collision + 1 / this->control_frequency_ * 1000 * 1.15)))
                    break;
                translation_feedback->distance_traveled = compute_distance_();
                if (goal->enable_collision_check)
                {
                    if (!check_translation_collision_(poses, linear_vel, &t_collision))
                        return;
                }
            }
            if (abs(translation_feedback->distance_traveled - abs(goal->target_x)) <= xy_tolerance)
            {
                stop_robot_(linear_vel);
                break;
            }
            action_translation_server_->publish_feedback(translation_feedback);
            if (goal->enable_data_save)
            {
                FillFile::fill_one_ligne(fd_for_times, i, static_cast<double>((get_clock()->now() - t_begin_for).nanoseconds()) * 1e-9);
                FillFile::fill_one_ligne(fd_r_s_times, i, static_cast<double>((get_clock()->now() - t_init_for).nanoseconds()) * 1e-9, odometry.get_timestamps().at(i));
            }
            i++;
        }
        stop_robot_(linear_vel);

        if (goal->enable_data_save)
        {
            FillFile::close_file(fd_for_times);
            FillFile::close_file(fd_r_s_times);
        }

        try {
            this->current_pose_ = this->get_current_pose_();
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(get_logger(), "Translation failed: %s", e.what());
            action_translation_server_->terminate_current();
            return;
        }

        translation_feedback->distance_traveled = compute_distance_();
        auto res = std::make_shared<Translation::Result>();
        res->total_elapsed_time.nanosec = int((get_clock()->now() - t_init).nanoseconds());
        auto error = abs(translation_feedback->distance_traveled - abs(goal->target_x));

        RCLCPP_INFO(get_logger(), "Expected Movement: %.4f\nDistance Traveled: %.4f\n Tolerance: %.4f\n Error: %.4f", 
                goal->target_x, translation_feedback->distance_traveled, xy_tolerance, error)

        if (error <= xy_tolerance) {
            RCLCPP_INFO(get_logger(), "Translation was a success!");
            action_translation_server_->succeeded_current(res) 
        } else {
            RCLCPP_INFO(get_logger(), "Translation failed!");
            action_rotation_server_->terminate_current();
        }
    }

    minimum_jerk::Pose MinimumJerkRos::get_current_pose_()
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

    bool MinimumJerkRos::is_obstacle_on_path_(std::vector<geometry_msgs::msg::PoseStamped> path)
    {
        geometry_msgs::msg::PoseStamped current_pose, current_pose_in_path_frame;
        current_pose.header.frame_id = "base_footprint";

        if (path.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Cannot check obstacle, no path was given!");
            return false;
        }

        if (!cmr_geometry_utils::transform_pose(this->buffer_, path[0].header.frame_id, current_pose, current_pose_in_path_frame, rclcpp::Duration::from_seconds(this->transform_tolerance_)))
        {
            RCLCPP_ERROR(get_logger(), "Failed to get current pose in [%s] frame", path[0].header.frame_id.c_str());
            return true;
        }

        auto closest_pose_rank = get_rank_of_closest_pose_in_path_(current_pose_in_path_frame, path);

        for (std::size_t k = closest_pose_rank; k < path.size(); ++k)
        {
            if (path[k].header.frame_id == "")
                throw std::invalid_argument("Evaluated a pose with no frame id passed");
            if (path[k].header.frame_id != current_pose_in_path_frame.header.frame_id)
            {
                RCLCPP_ERROR(get_logger(), "Failure: frame missmatch in obstacle detection... Path Frame: [%s] vs Pose Frame: [%s]",
                             path[k].header.frame_id.c_str(), current_pose_in_path_frame.header.frame_id.c_str());
                throw std::invalid_argument("Path availability is evaluated in the wrong frame");
            }

            // If the pose is outside of the collision checker horizon, we stop checking for collision
            if (cmr_geometry_utils::compute_distance_2d(current_pose_in_path_frame, path[k]) > scaled_obstacle_lookahead_distance_)
                break;

            if (!this->is_collision_free_(path[k]))
            {
                if (debug_trajectory_)
                {
                    visualization_msgs::msg::Marker circle_marker;
                    circle_marker.header.frame_id = "map";
                    circle_marker.header.stamp = get_clock()->now();
                    circle_marker.id = 0;
                    circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    circle_marker.action = visualization_msgs::msg::Marker::ADD;
                    circle_marker.pose = path[k].pose;
                    circle_marker.scale.x = 0.15;
                    circle_marker.scale.y = 0.15;
                    circle_marker.scale.z = 0.15;
                    circle_marker.color.r = 255;
                    circle_marker.color.g = 0;
                    circle_marker.color.b = 0;
                    circle_marker.color.a = 1.0f;
                    circle_marker.lifetime = rclcpp::Duration(0s);
                    circle_marker.frame_locked = false;
                    obstacles_marker_pub_->publish(circle_marker);
                }

                auto &clk = *get_clock();
                RCLCPP_ERROR_THROTTLE(get_logger(), clk, 1500, "Obstacle on the way, robot will stop until timeout.");
                return true;
            }
        }

        return false;
    }

    bool MinimumJerkRos::is_collision_free_(const geometry_msgs::msg::PoseStamped &pose)
    {
        auto request = std::make_shared<cmr_msgs::srv::IsCollisionFree::Request>();
        request->pose = pose;
        request->in_global_costmap = false;

        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("collision"), "collision server not available, the robot will stop...");
            return false;
        }

        auto result = collision_srv_client_->send_request(request);
        // Wait for the result.
        if (!result->success)
        {
            RCLCPP_ERROR(rclcpp::get_logger("collision"), "Failed to contact collision server");
            return false;
        }

        if (!result.get()->success)
        {
            RCLCPP_ERROR(rclcpp::get_logger("collision"), "Failed to get costmaps' status");
            return false;
        }

        return result.get()->is_collision_free;
    }

    std::size_t MinimumJerkRos::get_rank_of_closest_pose_in_path_(const geometry_msgs::msg::PoseStamped &pose, std::vector<geometry_msgs::msg::PoseStamped> path)
    {
        if (path.size() == 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get closest pose in path from current pose: path is empty");
            throw std::invalid_argument("Evaluated pose in empty path!");
        }

        std::size_t k;
        std::size_t closest_pose_rank = 0;

        for (k = 0; k < path.size(); ++k)
        {
            if (pose.header.frame_id != path[k].header.frame_id)
            {
                RCLCPP_ERROR(get_logger(), "Failure: frame missmatch in obstacle detection... Path Frame: [%s] vs Pose Frame: [%s]",
                             path[k].header.frame_id.c_str(), pose.header.frame_id.c_str());
                throw std::invalid_argument("Path availability is evaluated in the wrong frame");
            }

            if (cmr_geometry_utils::compute_distance_2d(pose, path[k]) < cmr_geometry_utils::compute_distance_2d(pose, path[closest_pose_rank]))
                closest_pose_rank = k;
        }

        return closest_pose_rank;
    }

    std::vector<geometry_msgs::msg::PoseStamped> MinimumJerkRos::convert_poses_to_posestampeds_(std::vector<minimum_jerk::Pose> poses)
    {
        std::vector<geometry_msgs::msg::PoseStamped> posestampeds;
        for (minimum_jerk::Pose &pose : poses)
        {
            bool res;
            geometry_msgs::msg::PoseStamped current, pose_in_base;
            current.pose.position.x = pose.get_x();
            current.pose.position.y = pose.get_y();
            current.pose.position.z = 0;
            current.pose.orientation = to_quaternion_(pose.get_theta());
            current.header.frame_id = "base_footprint";
            try
            {
                res = cmr_geometry_utils::transform_pose(this->buffer_, "odom", current, pose_in_base, rclcpp::Duration::from_seconds(this->transform_tolerance_));
            }

            catch (tf2::TransformException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "base_footprint to odom: %s", e.what());
                throw e;
            }
            if (!res)
            {
                std::vector<geometry_msgs::msg::PoseStamped> empty;
                return empty;
            }

            posestampeds.push_back(pose_in_base);
        }
        return posestampeds;
    }

    bool MinimumJerkRos::show_trajectory_(std::vector<geometry_msgs::msg::PoseStamped> poses)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        geometry_msgs::msg::PoseStamped current_pose_global;
        for (geometry_msgs::msg::PoseStamped pose : poses)
        {
            if (!cmr_geometry_utils::transform_pose(this->buffer_, "map", pose, current_pose_global, rclcpp::Duration::from_seconds(transform_tolerance_)))
            {
                RCLCPP_ERROR(get_logger(), "Failed to build path: transform from base to odom failed!");
                return false;
            }
            else
            {
                current_pose_global.header.stamp = get_clock()->now();
                path.poses.push_back(current_pose_global);
            }
        }
        trajectory_pub_->publish(path);
        goal_pose_pub_->publish(path.poses.at(path.poses.size() - 1));

        return true;
    }

    geometry_msgs::msg::Quaternion MinimumJerkRos::to_quaternion_(double yaw)
    {

        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw / 2.0);
        q.z = sin(yaw / 2.0);

        return q;
    }

    void MinimumJerkRos::stop_robot_(geometry_msgs::msg::Twist vel)
    {
        vel.linear.x = 0;
        vel.angular.z = 0;

        this->publisher_->publish(vel);
    }

    Trajectory MinimumJerkRos::compute_translation_velocities_(const std::shared_ptr<const minimum_jerk_msgs::action::Translate::Goal> &goal)
    {
        Pose pose_target = Pose(goal->target_x, 0, 0);
        Pose pose_start = Pose(0, 0, 0);
        double dist = abs(pose_target.get_x());
        double max_total = dist / goal->min_velocity;
        TrajectoryPlanner controller = TrajectoryPlanner(max_total, 1 / this->control_frequency_);
        Robot robot = Robot("Robot", max_total, controller, pose_start, pose_target, "tx");
        robot.generate_trajectory();

        auto odometry = robot.get_odometry();

        if (goal->enable_data_save)
        {
            FillFile::fill_file_translation<Pose>(odometry.get_timestamps(), odometry.get_poses(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/poses.txt");
            FillFile::fill_file_translation<Velocity>(odometry.get_timestamps(), odometry.get_velocities(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/velocities.txt");
            FillFile::fill_file_translation<Acceleration>(odometry.get_timestamps(), odometry.get_accelerations(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/accelerations.txt");
        }
        return odometry;
    }

    bool MinimumJerkRos::check_translation_collision_(std::vector<geometry_msgs::msg::PoseStamped> poses, geometry_msgs::msg::Twist linear_vel, double *t_collision)
    {
        auto start = std::chrono::system_clock::now();
        // If there's an obstacle between the current pose and the horizon distance on the pre-planned path, stop the robot
        // The robot stops for idle_timeout_ seconds maximum
        // If the path wasn't cleared in that time, the action fails, recovery is to be handled in the behavior trees

        while (is_obstacle_on_path_(poses))
        {
            if (action_translation_server_->is_cancel_requested())
            {
                RCLCPP_WARN(get_logger(), "Goal was canceled. Canceling.");
                stop_robot_(linear_vel);
                action_translation_server_->terminate_all();
                return false;
            }

            stop_robot_(linear_vel);
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;

            *t_collision = (end - start).count();

            if (elapsed_seconds.count() > idle_timeout_)
            {
                RCLCPP_ERROR(get_logger(), "Minimum Jerk Action timed out: an obstacle was in the way and was not moved.");
                action_translation_server_->terminate_all();
                return false;
            }
        }
        return true;
    }
    double MinimumJerkRos::compute_distance_()
    {
        geometry_msgs::msg::Point start;
        start.x = this->init_pose_.get_x();
        start.y = this->init_pose_.get_y();
        start.z = 0;
        geometry_msgs::msg::Point target;
        target.x = this->current_pose_.get_x();
        target.y = this->current_pose_.get_y();
        target.z = 0;
        return abs(cmr_geometry_utils::compute_distance_2d(start, target));
    }

    Trajectory MinimumJerkRos::compute_rotation_velocities_(const std::shared_ptr<const minimum_jerk_msgs::action::Rotate::Goal> &goal)
    {
        double target = goal->target_yaw;
        if (goal->target_yaw < -M_PI || goal->target_yaw > M_PI)
        {
            target -= 2 * M_PI * std::floor((target + M_PI) / (2 * M_PI));
        }
        Pose pose_target = Pose(0, 0, target);
        Pose pose_start = Pose(0, 0, 0);
        double dist = abs(pose_target.get_theta() - pose_start.get_theta());
        double max_total = dist / goal->min_velocity;
        TrajectoryPlanner controller = TrajectoryPlanner(max_total, 1 / this->control_frequency_);
        Robot robot = Robot("Robot", max_total, controller, pose_start, pose_target, "r");
        robot.generate_trajectory();

        auto odometry = robot.get_odometry();

        if (goal->enable_data_save)
        {
            FillFile::fill_file_rotation<Pose>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_poses(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/poses.txt");
            FillFile::fill_file_rotation<Velocity>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_velocities(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/velocities.txt");
            FillFile::fill_file_rotation<Acceleration>(robot.get_odometry().get_timestamps(), robot.get_odometry().get_accelerations(), "./src/mypackage/minimum_jerk_relative_move/minimum_jerk_trajectory_planner/ressources/accelerations.txt");
        }
        return odometry;
    }

    bool MinimumJerkRos::check_rotation_collision_(std::vector<geometry_msgs::msg::PoseStamped> poses, geometry_msgs::msg::Twist angular_vel, double *t_collision)
    {
        auto start = std::chrono::system_clock::now();
        // If there's an obstacle between the current pose and the horizon distance on the pre-planned path, stop the robot
        // The robot stops for idle_timeout_ seconds maximum
        // If the path wasn't cleared in that time, the action fails, recovery is to be handled in the behavior trees

        while (is_obstacle_on_path_(poses))
        {
            if (action_rotation_server_->is_cancel_requested())
            {
                RCLCPP_WARN(get_logger(), "Goal was canceled. Canceling.");
                stop_robot_(angular_vel);
                action_rotation_server_->terminate_all();
                return false;
            }

            stop_robot_(angular_vel);
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;

            *t_collision = (end - start).count();

            if (elapsed_seconds.count() > idle_timeout_)
            {
                RCLCPP_ERROR(get_logger(), "Minimum Jerk Action timed out: an obstacle was in the way and was not moved.");
                action_rotation_server_->terminate_all();
                return false;
            }
        }
        return true;
    }
    double MinimumJerkRos::compute_angle_()
    {
        double angle = abs(this->current_pose_.get_theta() - this->init_pose_.get_theta());
        angle -= 2 * M_PI * std::floor((angle + M_PI) / (2 * M_PI));
        angle = abs(angle);
        return angle;
    }
}