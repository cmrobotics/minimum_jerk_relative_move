#pragma once
#include "minimum_jerk_trajectory_planner/pose.hpp"
#include <math.h>
#include <stdlib.h>
#include <vector>

namespace minimum_jerk
{
class TrajectoryPlanner
{
public:
  std::vector<double> get_list_timestamps();
  std::vector<Pose> get_list_poses();
  TrajectoryPlanner(double total_time, double dt);

  void generate_trajectory(const Pose & init_pos, const Pose & target_pos);

private:
  std::vector<double> list_timestamps_;
  std::vector<Pose> list_poses_;
  double time_;
  double dt_;
  double calculate_trajectory_(double xi, double xf, double t);
};
}