#pragma once

namespace minimum_jerk
{
class Pose
{
public:
  double get_x() const;
  double get_y() const;
  double get_theta() const;
  Pose(double px = 0, double py = 0, double ptheta = 0);
  Pose operator-(Pose other);
  bool operator==(Pose other);

private:
  double x_;
  double y_;
  double theta_;
};
}