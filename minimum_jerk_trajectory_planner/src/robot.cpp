#include "minimum_jerk_trajectory_planner/robot.hpp"

namespace minimum_jerk
{

    Robot::Robot(char *name, double total_time, TrajectoryPlanner &path_finder_controller, Pose& pose_start, Pose& pose_target, char *move_type) : name(name), path_finder_controller(path_finder_controller), total_time(total_time), pose(pose_start), pose_start(pose_start), pose_target(pose_target), move_type(move_type)
    {
        current_step = 0;
        is_at_target = false;
        odometry = nullptr;

        if (strcmp(move_type, "r") && strcmp(move_type, "tx") && strcmp(move_type, "ty"))
        {
            throw "Couldn't build Robot instance, move type has to be 'r' or 'tx' or 'ty' (rotate or translate)";
        }
    }
    Robot::~Robot()
    {
        if (odometry != nullptr)
        {
            delete odometry;
        }
    }
    Robot::Robot(const Robot &src): path_finder_controller(src.path_finder_controller), pose(src.pose_start), pose_start(src.pose), pose_target(src.pose_target)
    {
        char * move_type = "";
        strcpy(move_type, src.move_type);
        odometry = nullptr;
    }
    Robot &Robot::operator=(const Robot &src)
    {
        strcpy(move_type, src.move_type);
        name = name;
        total_time = total_time;
        path_finder_controller = path_finder_controller;
        pose_start = pose_start;
        pose_target = pose_start;
        move_type = move_type;
        return *this;
    }

    void Robot::generate_trajectory(double dt)
    {
        path_finder_controller.generate_trajectory(pose_start,pose_target);
        odometry= new Trajectory(path_finder_controller.list_t, path_finder_controller.list_pose,total_time/dt);
    }
    bool Robot::operator==(const Robot &r) const
    {
        return name == r.name && pose_start.x == r.pose_start.x && pose_start.y == r.pose_start.y && pose_start.theta == r.pose_start.theta;
    }
    bool Robot::operator!=(const Robot &r) const
    {
        return !(*this == r);
    }
}