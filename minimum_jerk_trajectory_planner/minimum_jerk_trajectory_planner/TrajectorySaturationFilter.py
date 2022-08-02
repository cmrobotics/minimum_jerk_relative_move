from .Trajectory import Trajectory

class TrajectorySaturation:
    def __init__(self, max_angular_vel, max_angular_accel, max_linear_vel, max_linear_accel):
        self.max_angular_vel = max_angular_vel
        self.max_angular_accel = max_angular_accel
        self.max_linear_vel = max_linear_vel
        self.max_linear_accel = max_linear_accel
    
    def saturate_trajectory(self, trajectory):
        poses = trajectory["poses"]
        vels = trajectory["velocities"]
        accels = trajectory["accelerations"]
        return