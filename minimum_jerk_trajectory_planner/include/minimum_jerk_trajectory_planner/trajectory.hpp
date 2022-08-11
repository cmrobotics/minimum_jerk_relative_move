#pragma once
#include "acceleration.hpp"
#include "velocity.hpp"
#include "pose.hpp"
#include <stdlib.h>
#include <string.h>

namespace minimum_jerk
{
    class Trajectory
    {
    public:
        double* timestamps;
        Pose *poses;
        int poses_size;
        Velocity *velocitoties;
        Acceleration *accelerations;
        Trajectory(double* timestamps, Pose *poses, int poses_size);
        ~Trajectory();
        Trajectory(const Trajectory &src);
        Trajectory &operator=(const Trajectory &src);

    private:
        void set_velocities(Velocity* vel);
        void set_accelerations(Acceleration* acc);
    };
}