#include "minimum_jerk_trajectory_planner/trajectory.hpp"

namespace minimum_jerk
{
Trajectory::Trajectory(std::vector<double> timestamps, std::vector<Pose> poses) : timestamps_(timestamps), poses_(poses)
{
  set_velocities_();
  set_accelerations_();
}

void Trajectory::set_velocities_()
{
  double dt;
  for (size_t k = 0; k < poses_.size() - 1; k++) {
    dt = timestamps_.at(k + 1) - timestamps_.at(k);
    velocities_.push_back(Velocity((poses_.at(k + 1).get_x() - poses_.at(k).get_x()) / dt,
                                   (poses_.at(k + 1).get_y() - poses_.at(k).get_y()) / dt,
                                   (poses_.at(k + 1).get_theta() - poses_.at(k).get_theta()) / dt));
  }
}

void Trajectory::set_accelerations_()
{
  double dt;
  for (size_t k = 0; k < velocities_.size() - 1; k++) {
    dt = timestamps_.at(k + 1) - timestamps_.at(k);
    accelerations_.push_back(Acceleration((velocities_.at(k + 1).get_x() - velocities_.at(k).get_x()) / dt,
                                          (velocities_.at(k + 1).get_y() - velocities_.at(k).get_y()) / dt,
                                          (velocities_.at(k + 1).get_theta() - velocities_.at(k).get_theta()) / dt));
  }
}
std::vector<double> Trajectory::get_timestamps() const
{
  return timestamps_;
}
std::vector<Pose> Trajectory::get_poses() const
{
  return poses_;
}
std::vector<Velocity> Trajectory::get_velocities() const
{
  return velocities_;
}
std::vector<Acceleration> Trajectory::get_accelerations() const
{
  return accelerations_;
}

}