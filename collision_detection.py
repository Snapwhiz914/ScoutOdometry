#reports collision

class CollisionDetector():
    def __init__(self, threshold: float = 0.05):
        self.prev_cmd = None
        self.t = threshold
    
    def loop(self, accel_data, new_cmd):
        if self.prev_cmd == None:
            self.prev_cmd = new_cmd
            return False
        if self.prev_cmd != new_cmd:
            return False
        else:
            if abs(accel_data.y) > self.t: return True #since accel is change in vel, if abs(change in vel) + t is greater than the set vel, then large change just happened
        self.prev_cmd = new_cmd
        return False