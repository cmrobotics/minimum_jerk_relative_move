#include "minimum_jerk_trajectory_planner/acceleration.hpp"

namespace minimum_jerk
{
    Acceleration::Acceleration(double ax, double ay, double atheta)
    {
        x = ax;
        y = ay;
        theta = atheta;
    }
}