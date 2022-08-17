#pragma once
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"
#include "minimum_jerk_trajectory_planner/trajectory.hpp"
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include <memory>

namespace minimum_jerk
{
    class Robot
    {
    public:
        Robot(std::string name, double total_time, const TrajectoryPlanner & path_finder_controller, const Pose & pose_start, const Pose & pose_target, std::string move_type);
        Robot(const Robot &src);
        Robot &operator=(const Robot &src);

        void generate_trajectory();
        bool operator==(const Robot &r) const;
        bool operator!=(const Robot &r) const;

        Pose get_pose() const;
        Pose get_pose_start() const;
        Pose get_pose_target() const;
        TrajectoryPlanner get_path_finder_controller() const;
        std::string get_move_type() const;
        std::string get_name() const;
        double get_total_time() const;
        int get_current_step() const;
        bool get_is_at_target() const;
        Trajectory get_odometry() const;

    private:
        std::string name_;
        std::unique_ptr<TrajectoryPlanner> path_finder_controller_;
        double total_time_;
        std::unique_ptr<Pose> pose_;
        std::unique_ptr<Pose> pose_start_;
        std::unique_ptr<Pose> pose_target_;
        int current_step_;
        bool is_at_target_;
        std::unique_ptr<Trajectory> odometry_;
        std::string move_type_;
    };
}