#include "minimum_jerk_trajectory_planner/pose.hpp"
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"
#include "minimum_jerk_trajectory_planner/robot.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace minimum_jerk;

int main()
{

    TrajectoryPlanner controller = TrajectoryPlanner();
    Pose pose_target = Pose(5, 5, 0);
    Pose pose_strat = Pose(3, 5, 0);
    int dist = abs(pose_target.x - pose_strat.x);
    double max_linear_velocity = 0.0;
    double max_total = dist / max_linear_velocity;
    Robot robot = Robot((char *)"Robot", max_total, controller, pose_strat, pose_target, (char *)"tx");
    robot.generate_trajectory(0.01);
    printf("pose :  %f %f %f\n", robot.odometry->poses[robot.odometry->poses_size - 1].x, robot.odometry->poses[robot.odometry->poses_size - 1].y, robot.odometry->poses[robot.odometry->poses_size - 1].theta);
    return 0;
}
