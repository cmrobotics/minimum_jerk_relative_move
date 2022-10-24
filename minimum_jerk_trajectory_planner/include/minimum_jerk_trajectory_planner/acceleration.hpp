#pragma once

namespace minimum_jerk
{
class Acceleration
{
public:
  double get_x() const;
  double get_y() const;
  double get_theta() const;
  Acceleration(double ax, double ay, double atheta);

private:
  double x_;
  double y_;
  double theta_;
};
}