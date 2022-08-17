#pragma once

namespace minimum_jerk
{
    class Velocity
    {
    public:
        double get_x() const;
        double get_y() const;
        double get_theta() const;
        Velocity(double vx, double vy, double vtheta);

    private:
        double x_;
        double y_;
        double theta_;
    };
}