#pragma once
#include "acceleration.hpp"
#include "velocity.hpp"
#include "pose.hpp"
#include <stdlib.h>
#include <string.h>
#include <vector>


namespace minimum_jerk
{
    class Trajectory
    {
    public:
        std::vector<double> timestamps;
        std::vector<Pose> poses;
        std::vector<Velocity> velocities;
        std::vector<Acceleration> accelerations;
        Trajectory(std::vector<double>  timestamps, std::vector<Pose> poses);

    private:
        void set_velocities();
        void set_accelerations();
    };
}