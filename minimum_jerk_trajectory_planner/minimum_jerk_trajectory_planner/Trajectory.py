from .Velocity import Velocity
from .Acceleration import Acceleration

class Trajectory:
    def __init__(self, timestamps, poses):
        self.timestamps = timestamps
        self.poses = poses
        self.velocities = self.__set_velocities()
        self.accelerations = self.__set_acceleration()

    def __set_velocities(self):
        vels = [Velocity(0, 0, 0)]
        for k in range(len(self.poses)):
            if k+1 > len(self.poses) - 1:
                break

            dt = self.timestamps[k+1] - self.timestamps[k]
            vx = (self.poses[k+1].x - self.poses[k].x) / dt
            vy = (self.poses[k+1].y - self.poses[k].y) / dt
            vtheta = (self.poses[k+1].theta - self.poses[k].theta) / dt
            
            v = Velocity(vx, vy, vtheta)
            vels.append(v)

        return vels

    def __set_acceleration(self):
        accels = [Acceleration(0, 0, 0)]
        for k in range(len(self.velocities)):
            if k+1 > len(self.velocities) - 1:
                break

            dt = self.timestamps[k+1] - self.timestamps[k]
            ax = (self.velocities[k+1].x - self.velocities[k].x) / dt
            ay = (self.velocities[k+1].y - self.velocities[k].y) / dt
            atheta = (self.velocities[k+1].theta - self.velocities[k].theta) / dt
            
            a = Acceleration(ax, ay, atheta)
            accels.append(a)

        return accels
