#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"


namespace minimum_jerk
{
    TrajectoryPlanner::TrajectoryPlanner(double total_time , double dt){
        time =total_time;
        this->dt = dt;
        list_t = (double*)malloc(sizeof(double)*(time/dt));
        list_pose = (Pose*)malloc(sizeof(Pose)*(time/dt));
    }

    TrajectoryPlanner::~TrajectoryPlanner(){
        free(list_t);
        free(list_pose);
    }

    TrajectoryPlanner::TrajectoryPlanner(const TrajectoryPlanner& src): time(src.time), dt(src.dt){
        list_t = (double*)malloc(sizeof(double)*(time/dt));
        list_pose = (Pose*)malloc(sizeof(Pose)*(time/dt));
        memcpy(list_t, src.list_t, time/dt*sizeof(double));
        memcpy(list_pose, src.list_pose, time/dt*sizeof(Pose));
    }

    TrajectoryPlanner& TrajectoryPlanner::operator=(const TrajectoryPlanner& src){
        time = src.time;
        dt = src.dt;
        list_t = (double*)malloc(sizeof(double)*(time/dt));
        list_pose = (Pose*)malloc(sizeof(Pose)*(time/dt));
        memcpy(list_t, src.list_t, time/dt*sizeof(double));
        memcpy(list_pose, src.list_pose, time/dt*sizeof(Pose));
        return *this;
    }

    void TrajectoryPlanner::generate_trajectory(Pose init_pos, Pose target_pos)
    {
        double t = 0;
        int i = 0;
        while (t < time)
        {
            Pose pos = Pose(trajectory_calculation(init_pos.x, target_pos.x, t),
                            trajectory_calculation(init_pos.y, target_pos.y, t),
                            trajectory_calculation(init_pos.theta, target_pos.theta, t));
            list_t[i] = t;
            list_pose[i] = pos;
            i++;
            t += dt;
        }
    }
    double TrajectoryPlanner::trajectory_calculation(double xi, double xf, double t)
    {
        return xi + (xf - xi) * (10 * pow((t / time), 3) - 15 * pow((t / time), 4) + 6 * pow((t / time), 5));
    }
}