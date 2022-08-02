class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def __sub__(self, other):
        return Pose(self.x - other.x,
                    self.y - other.y,
                    self.theta - other.theta)