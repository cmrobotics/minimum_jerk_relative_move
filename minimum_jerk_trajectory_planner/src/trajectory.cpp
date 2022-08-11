#include "minimum_jerk_trajectory_planner/trajectory.hpp"

namespace minimum_jerk
{
    Trajectory::Trajectory(double *timestamps, Pose *poses, int poses_size) : timestamps(timestamps), poses(poses), poses_size(poses_size)
    {
        velocitoties = (Velocity *)malloc(sizeof(Velocity) * poses_size - 1);
        accelerations = (Acceleration *)malloc(sizeof(Acceleration) * poses_size - 2);
        set_velocities(velocitoties);
        set_accelerations(accelerations);
    }

    Trajectory::~Trajectory()
    {
        free(velocitoties);
        free(accelerations);
    }

    Trajectory::Trajectory(const Trajectory &src) : timestamps(src.timestamps), poses_size(src.poses_size)
    {
        velocitoties = (Velocity *)malloc(sizeof(Velocity) * poses_size - 1);
        accelerations = (Acceleration *)malloc(sizeof(Acceleration) * poses_size - 2);
        memcpy(velocitoties, src.velocitoties, poses_size * sizeof(Velocity));
        memcpy(accelerations, src.accelerations, poses_size * sizeof(Acceleration));
    }

    Trajectory &Trajectory::operator=(const Trajectory &src)
    {
        velocitoties = (Velocity *)malloc(sizeof(Velocity) * poses_size - 1);
        accelerations = (Acceleration *)malloc(sizeof(Acceleration) * poses_size - 2);
        timestamps = src.timestamps;
        poses_size = src.poses_size;
        memcpy(velocitoties, src.velocitoties, poses_size * sizeof(Velocity));
        memcpy(accelerations, src.accelerations, poses_size * sizeof(Acceleration));
        return *this;
    }

    void Trajectory::set_velocities(Velocity *vel)
    {
        double dt;
        for (int k = 0; k < poses_size - 1; k++)
        {
            dt = timestamps[k + 1] - timestamps[k];
            velocitoties[k] = Velocity((poses[k].x - poses[k + 1].x) / dt,
                                       (poses[k].y - poses[k + 1].y) / dt,
                                       (poses[k].theta - poses[k + 1].theta) / dt);
        }
    }

    void Trajectory::set_accelerations(Acceleration *acc)
    {
        double dt;
        for (int k = 0; k < poses_size - 2; k++)
        {
            dt = timestamps[k + 1] - timestamps[k];
            velocitoties[k] = Velocity((velocitoties[k].x - velocitoties[k + 1].x) / dt,
                                       (velocitoties[k].y - velocitoties[k + 1].y) / dt,
                                       (velocitoties[k].theta - velocitoties[k + 1].theta) / dt);
        }
    }

}