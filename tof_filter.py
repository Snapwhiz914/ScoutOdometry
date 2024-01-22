import numpy as np

TOO_FAR = 0.89 # 35 in

class TOFFilter:
    def __init__(self):
        self.last_values = [0.0, 0.0, 0.0, 0.0]
        self.first_run = True
        self.weights = [0.25, 0.5, 0.75, 1.0]
        self.too_close = 0.0
        self.too_far = TOO_FAR
    
    def loop(self, tof_data):
        if self.first_run:
            self.too_close = tof_data.min_range
            
            ta = tof_data.range
            if float(ta) < self.too_close: to_add = self.too_close
            if float(ta) > self.too_far: to_add = self.too_far
            
            self.last_values = [ta for i in range(len(self.weights))]
            self.first_run = False
        
        if len(self.last_values) >= len(self.weights): self.last_values.pop(0)

        to_add = tof_data.range
        if float(to_add) < self.too_close: to_add = self.too_close
        if float(to_add) > self.too_far: to_add = self.too_far
        
        self.last_values.append(to_add)

        return np.average(self.last_values, weights=self.weights)