from .Pose import Pose
from .TrajectoryPlanners import MinimumJerkTrajectoryPlanner
from .Robot import Robot
from .simulation import run_simulation

def main():
    controller = MinimumJerkTrajectoryPlanner()
    pose_target1 = Pose(2, 2, -3.14)
    pose_target2 = Pose(5, 5, 0)
    pose_target3 = Pose(8, 8, 1.57)

    pose_start_1 = Pose(2, 2, 0)
    pose_start_2 = Pose(3, 5, 0)
    pose_start_3 = Pose(8, 3, 1.57)

    dist1 = abs(pose_target1.theta - pose_start_1.theta)
    dist2 = abs(pose_target2.x - pose_start_2.x)
    dist3 = abs(pose_target3.y - pose_start_3.y)

    max_angular_vel = 1.0
    max_linear_vel = 0.8

    max_total_time1 = dist1/max_angular_vel
    max_total_time2 = dist2/max_linear_vel
    max_total_time3 = dist3/max_linear_vel

    robot_1 = Robot("Yellow Robot", "y", max_total_time1, controller, pose_start_1, pose_target1, "r")
    robot_2 = Robot("Black Robot", "k", max_total_time2, controller, pose_start_2, pose_target2, "tx")
    robot_3 = Robot("Blue Robot", "b", max_total_time3, controller, pose_start_3, pose_target3, "ty")

    robots: list[Robot] = [robot_1, robot_2, robot_3]

    run_simulation(robots, max(max_total_time1, max_total_time2, max_total_time3) + 0.5)


if __name__ == '__main__':
    main()