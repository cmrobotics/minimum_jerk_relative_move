class Pose:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def __sub__(self, other):
        return Pose(self.x - other.x,
                    self.y - other.y,
                    self.theta - other.theta)