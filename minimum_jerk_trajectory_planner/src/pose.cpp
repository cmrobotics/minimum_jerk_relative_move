#include "minimum_jerk_trajectory_planner/pose.hpp"

namespace minimum_jerk
{
    Pose::Pose(double px, double py, double ptheta)
    {
        x_ = px;
        y_ = py;
        theta_ = ptheta;
    }
    Pose Pose::operator-(Pose other)
    {
        return Pose(this->x_ - other.x_, this->y_ - other.y_, this->theta_ - other.theta_);
    }
    bool Pose::operator==(Pose other)
    {
        return (this->x_ == other.x_ && this->y_ == other.y_ && this->theta_ == other.theta_);
    }
    double Pose::get_x() const
    {
        return x_;
    }
    double Pose::get_y() const
    {
        return y_;
    }
    double Pose::get_theta() const
    {
        return theta_;
    }
}