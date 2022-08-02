from .Trajectory import Trajectory
from .Pose import Pose

class Robot:
    def __init__(self, name, color, total_time,
                 path_finder_controller, pose_start: Pose, pose_target: Pose, move_type):
        self.name = name
        self.color = color
        self.path_finder_controller = path_finder_controller
        self.x_traj = []
        self.y_traj = []

        self.total_time = total_time

        self.pose = pose_start
        self.pose_start = pose_start
        self.pose_target = pose_target

        self.trajectory = None
        self.current_step = 0
        self.is_at_target = False

        if move_type != "r" and move_type != "tx" and move_type != "ty":
            raise Exception("Couldn't build Robot instance, move type has to be 'r' or 'tx' or 'ty' (rotate or translate)")
        
        self.move_type = move_type

    def generate_trajectory(self, dt):
        self.trajectory = self.path_finder_controller.generate_trajectory(self.pose_start, self.pose_target, self.total_time, dt)
        self.odometry = Trajectory(self.trajectory[0], self.trajectory[1])
        return self.trajectory

    def move(self):
        try:
            if not self.trajectory:
                print("You tried to move without generating a trajectory")
                return

            if self.is_at_target:
                self.x_traj.append(self.pose.x)
                self.y_traj.append(self.pose.y)
                return

            self.x_traj.append(self.pose.x)
            self.y_traj.append(self.pose.y)

            self.pose.theta = self.trajectory[1][self.current_step].theta
            self.pose.x = self.trajectory[1][self.current_step].x
            self.pose.y = self.trajectory[1][self.current_step].y

            self.current_step += 1
            
            if self.current_step >= len(self.trajectory[1]) - 1:
                self.is_at_target = True

        except IndexError as e:
            print(f"Robot {self.name}'s software crashed")
            raise

    def __eq__(self, other):
        return (self.name, self.color, self.pose_start.x, self.pose_start.y, self.pose_start.theta) == (other.name, other.color, other.pose_start.x, other.pose_start.y, other.pose_start.theta)
    
    def __hash__(self):
        return hash((self.name, self.color, self.pose_start.x, self.pose_start.y, self.pose_start.theta))

    def __ne__(self, other):
        return not(self == other)