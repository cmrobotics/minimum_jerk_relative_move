#include "minimum_jerk_trajectory_planner/velocity.hpp"

namespace minimum_jerk
{
    Velocity::Velocity(double vx, double vy, double vtheta)
    {
        x = vx;
        y = vy;
        theta = vtheta;
    }
}