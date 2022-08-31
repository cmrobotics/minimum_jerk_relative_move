#pragma once
#include "minimum_jerk_trajectory_planner/acceleration.hpp"
#include "minimum_jerk_trajectory_planner/velocity.hpp"
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include <stdlib.h>
#include <vector>

namespace minimum_jerk
{
class Trajectory
{
public:
    std::vector<double> get_timestamps() const;
    std::vector<Pose> get_poses() const;
    std::vector<Velocity> get_velocities() const;
    std::vector<Acceleration> get_accelerations() const;
    Trajectory(std::vector<double> timestamps, std::vector<Pose> poses);

private:
    std::vector<double> timestamps_;
    std::vector<Pose> poses_;
    std::vector<Velocity> velocities_;
    std::vector<Acceleration> accelerations_;
    void set_velocities_();
    void set_accelerations_();
};
}