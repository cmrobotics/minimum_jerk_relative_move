#include "minimum_jerk_trajectory_planner/trajectory.hpp"

namespace minimum_jerk
{
    Trajectory::Trajectory(std::vector<double> timestamps, std::vector<Pose> poses) : timestamps(timestamps), poses(poses)
    {
        set_velocities();
        set_accelerations();
    }

    void Trajectory::set_velocities()
    {
        double dt;
        for (size_t k = 0; k < poses.size() - 1; k++)
        {
            dt = timestamps.at(k + 1) - timestamps.at(k);
            velocities.push_back(Velocity((poses.at(k + 1).x - poses.at(k).x) / dt,
                                            (poses.at(k + 1).y - poses.at(k).y) / dt,
                                            (poses.at(k + 1).theta - poses.at(k).theta) / dt));
        }
    }

    void Trajectory::set_accelerations()
    {
        double dt;
        for (size_t k = 0; k < velocities.size() - 1; k++)
        {
            dt = timestamps.at(k + 1) - timestamps.at(k);
            accelerations.push_back(Acceleration((velocities.at(k + 1).x - velocities.at(k).x) / dt,
                                                 (velocities.at(k + 1).y - velocities.at(k).y) / dt,
                                                 (velocities.at(k + 1).theta - velocities.at(k).theta) / dt));
        }
    }

}