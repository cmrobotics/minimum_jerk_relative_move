#include "minimum_jerk_trajectory_planner/pose.hpp"

namespace minimum_jerk
{
    Pose::Pose(double ax, double ay, double atheta)
    {
        x = ax;
        y = ay;
        theta = atheta;
    }
    Pose Pose::operator-(Pose other){
        return Pose(this->x - other.x, this->y - other.y, this->theta - other.theta); 
    }
}