#pragma once

namespace minimum_jerk
{
    class Pose
    {
    public:
        double x;
        double y;
        double theta;
        Pose(double ax = 0, double ay = 0, double atheta = 0);
        Pose operator-(Pose other);
    };
}