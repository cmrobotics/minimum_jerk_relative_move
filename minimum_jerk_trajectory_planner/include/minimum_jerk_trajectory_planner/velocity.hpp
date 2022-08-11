#pragma once

namespace minimum_jerk
{
    class Velocity
    {
    public:
        double x;
        double y;
        double theta;
        Velocity(double vx, double vy, double vtheta);
    };
}