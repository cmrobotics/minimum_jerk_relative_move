#pragma once
#include "pose.hpp"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>


namespace minimum_jerk
{
    class TrajectoryPlanner
    {
    public:
        std::vector<double> list_t;
        std::vector<Pose> list_pose;
        double time;
        double dt;
        TrajectoryPlanner(double total_time = 0.5, double dt = 0.01);

        void generate_trajectory(Pose init_pos, Pose target_pos);

        private:
            double trajectory_calculation(double xi, double xf, double t);
    };
}