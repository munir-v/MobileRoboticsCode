class Pose:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}"