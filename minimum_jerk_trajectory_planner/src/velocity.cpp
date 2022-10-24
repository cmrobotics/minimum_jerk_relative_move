#include "minimum_jerk_trajectory_planner/velocity.hpp"

namespace minimum_jerk
{
Velocity::Velocity(double vx, double vy, double vtheta)
{
  x_ = vx;
  y_ = vy;
  theta_ = vtheta;
}
double Velocity::get_x() const
{
  return x_;
}
double Velocity::get_y() const
{
  return y_;
}
double Velocity::get_theta() const
{
  return theta_;
}
}