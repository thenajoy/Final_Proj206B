import numpy as np

class Arena(object):
    def __init__(self):
        self.rosservice_timeout = 2
        self.arena_lb = np.array([-2.5, -2.75, -0.01])
        self.arena_ub = np.array([1.75, 2.5, 3])
    def is_valid_position(self, position):
        flag = True
        for i in range(3):
            flag = flag and (position[i] <= self.arena_ub[i]) and (position[i] >= self.arena_lb[i])
        return flag
