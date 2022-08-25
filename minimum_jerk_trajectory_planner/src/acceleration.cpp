#include "minimum_jerk_trajectory_planner/acceleration.hpp"

namespace minimum_jerk
{
Acceleration::Acceleration(double ax, double ay, double atheta)
{
    x_ = ax;
    y_ = ay;
    theta_ = atheta;
}
double Acceleration::get_x() const
{
    return x_;
}
double Acceleration::get_y() const
{
    return y_;
}
double Acceleration::get_theta() const
{
    return theta_;
}
}