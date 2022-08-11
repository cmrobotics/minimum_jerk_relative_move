#pragma once

namespace minimum_jerk
{
    class Acceleration
    {
    public:
        double x;
        double y;
        double theta;
        Acceleration(double ax, double ay, double atheta);
    };
}