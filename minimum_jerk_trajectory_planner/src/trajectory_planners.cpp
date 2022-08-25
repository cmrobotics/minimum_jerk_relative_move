#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"

namespace minimum_jerk
{
TrajectoryPlanner::TrajectoryPlanner(double total_time, double dt)
{
    time_ = total_time;
    this->dt_ = dt;
}

void TrajectoryPlanner::generate_trajectory(const Pose &init_pos, const Pose &target_pos)
{
    double t = 0;
    int i = 0;
    while (t < time_)
    {
        Pose pos = Pose(calculate_trajectory_(init_pos.get_x(), target_pos.get_x(), t),
                        calculate_trajectory_(init_pos.get_y(), target_pos.get_y(), t),
                        calculate_trajectory_(init_pos.get_theta(), target_pos.get_theta(), t));
        list_timestamps_.push_back(t);
        list_poses_.push_back(pos);
        i++;
        t += dt_;
    }
}
double TrajectoryPlanner::calculate_trajectory_(double xi, double xf, double t)
{
    return xi + (xf - xi) * (10 * pow((t / time_), 3) - 15 * pow((t / time_), 4) + 6 * pow((t / time_), 5));
}
std::vector<double> TrajectoryPlanner::get_list_timestamps()
{
    return list_timestamps_;
}
std::vector<Pose> TrajectoryPlanner::get_list_poses()
{
    return list_poses_;
}
}