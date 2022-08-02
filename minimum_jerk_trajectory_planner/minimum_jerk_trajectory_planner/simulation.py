import numpy as np
import matplotlib.pyplot as plt

TIME_STEP = 0.1
AT_TARGET_ACCEPTANCE_THRESHOLD = 0.01
SHOW_ANIMATION = True
SHOW_TRAJECTORY = True
PLOT_WINDOW_SIZE_X = 10
PLOT_WINDOW_SIZE_Y = 10
PLOT_FONT_SIZE = 8

simulation_running = True
all_robots_are_at_target = False

def run_simulation(robots, time_duration):
    """Simulates all robots simultaneously"""
    global all_robots_are_at_target
    global simulation_running

    trajectories = dict()

    robot_names = []
    for instance in robots:
        robot_names.append(instance.name)
        trajectories[instance] = instance.generate_trajectory(TIME_STEP)

    time = 0
    while simulation_running and time < time_duration:
        time += TIME_STEP
        robots_are_at_target = []

        for instance in robots:
            if not instance.is_at_target:
                instance.move()
            robots_are_at_target.append(instance.is_at_target)

        if all(robots_are_at_target):
            simulation_running = False

        if SHOW_ANIMATION:
            plt.cla()
            plt.xlim(0, PLOT_WINDOW_SIZE_X)
            plt.ylim(0, PLOT_WINDOW_SIZE_Y)

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 1,
                     'Time: {:.2f}'.format(time),
                     fontsize=PLOT_FONT_SIZE)

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 2,
                     'Reached target: {} = '.format(robot_names)
                     + str(robots_are_at_target),
                     fontsize=PLOT_FONT_SIZE)

            for instance in robots:
                plt.arrow(instance.pose_start.x,
                          instance.pose_start.y,
                          np.cos(instance.pose_start.theta),
                          np.sin(instance.pose_start.theta),
                          color='r',
                          width=0.1)
                plt.arrow(instance.pose_target.x,
                          instance.pose_target.y,
                          np.cos(instance.pose_target.theta),
                          np.sin(instance.pose_target.theta),
                          color='g',
                          width=0.1)
                plot_vehicle(instance.pose.x,
                             instance.pose.y,
                             instance.pose.theta,
                             instance.x_traj,
                             instance.y_traj, instance.color)

            plt.pause(TIME_STEP)

    if SHOW_TRAJECTORY:
        for k in trajectories:
            fig, axes = plt.subplots()
            new_poses = []
            new_accel = []
            new_vel = []

            if k.move_type == "r":
                for pose in k.odometry.poses:
                    new_poses.append(pose.theta)
                
                for v in k.odometry.velocities:
                    new_vel.append(v.theta)
                
                for a in k.odometry.accelerations:
                    new_accel.append(a.theta)

            elif k.move_type == "ty":
                for pose in k.odometry.poses:
                    new_poses.append(pose.y)
                
                for v in k.odometry.velocities:
                    new_vel.append(v.y)
                
                for a in k.odometry.accelerations:
                    new_accel.append(a.y)

            elif k.move_type == "tx":
                for pose in k.odometry.poses:
                    new_poses.append(pose.x)

                for v in k.odometry.velocities:
                    new_vel.append(v.x)
                
                for a in k.odometry.accelerations:
                    new_accel.append(a.x)

            unit = "rad" if k.move_type == "r" else "m"
            axes.plot(k.odometry.timestamps, new_poses)
            axes.plot(k.odometry.timestamps, new_vel)
            axes.plot(k.odometry.timestamps, new_accel)
            axes.legend([f"Pose ({unit})", 
                         f"Velocity ({unit}/s)",
                         f"Acceleration ({unit}/s²)"
                        ])
            fig.suptitle("Trajectory Course")
        
        plt.show()


def plot_vehicle(x, y, theta, x_traj, y_traj, color):
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = T @ p1_i
    p2 = T @ p2_i
    p3 = T @ p3_i

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color+'-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color+'-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color+'-')

    plt.plot(x_traj, y_traj, color+'--')


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])
