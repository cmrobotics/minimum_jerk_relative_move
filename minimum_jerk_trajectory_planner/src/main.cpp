#include "minimum_jerk_trajectory_planner/pose.hpp"
#include "minimum_jerk_trajectory_planner/trajectory_planners.hpp"
#include "minimum_jerk_trajectory_planner/robot.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

using namespace minimum_jerk;

int main()
{
    TrajectoryPlanner controller = TrajectoryPlanner();
    Pose pose_target = Pose(5, 5, 3.14);
    Pose pose_strat = Pose(5, 5, 0);
    int dist = abs(pose_target.theta - pose_strat.theta);
    double max_linear_velocity = 0.0;
    double max_total = dist / max_linear_velocity;
    Robot robot = Robot((char *)"Robot", max_total, controller, pose_strat, pose_target, (char *)"r");
    robot.generate_trajectory();
    return 0;
}