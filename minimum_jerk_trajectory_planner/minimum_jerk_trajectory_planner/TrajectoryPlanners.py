import numpy as np
from .Pose import Pose

class MinimumJerkTrajectoryPlanner:
    def __init__(self):
        pass
    
    def generate_trajectory(self, init_pos, target_pos, total_time=0.5, dt=0.01):
        xi = np.array([init_pos.x, init_pos.y, init_pos.theta])
        xf = np.array([target_pos.x, target_pos.y, target_pos.theta])
        d = total_time
        list_t = []
        list_x = []
        t = 0
        while t < d:
            x = xi + (xf-xi) * (10*(t/d)**3 - 15*(t/d)**4 + 6*(t/d)**5)
            list_t.append(t)
            list_x.append(x)
            t += dt
        
        list_pose = []
        for pose in list_x:
            list_pose.append(Pose(pose[0], pose[1], pose[2]))
        return list_t, list_pose