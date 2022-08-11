#pragma once
#include "trajectory_planners.hpp"
#include "trajectory.hpp"
#include "pose.hpp"
#include <string.h>
#include<memory> 


namespace minimum_jerk
{
    class Robot
    {
    public:
        char *name;
        TrajectoryPlanner path_finder_controller;
        double total_time;
        Pose& pose;
        Pose& pose_start;
        Pose& pose_target;
        int current_step;
        bool is_at_target;
        std::unique_ptr<Trajectory> odometry;
        char *move_type;

        Robot(char *name, double total_time, TrajectoryPlanner& path_finder_controller, Pose& pose_start, Pose& pose_target, char *move_type);
        Robot(const Robot &src);
        Robot &operator=(const Robot &src);

        void generate_trajectory();
        bool operator==(const Robot &r) const;
        bool operator!=(const Robot &r) const;
    };
}