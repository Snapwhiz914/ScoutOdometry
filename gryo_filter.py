import time
import numpy as np
from ahrs.filters import Madgwick

KF_WEIGHT = 0.85

class GyroFilter():
    def __init__(self):
        self.last_time_seconds = 0
        self.this_time_seconds = round(time.time(), 3)
        self.last_angle = 0
        self.first_run = False
        self.last_values = [0.0, 0.0, 0.0, 0.0]
        self.weights = [0.25, 0.5, 0.75, 1.0]
    
    def loop(self, imu_data):
        self.last_time_seconds = self.this_time_seconds
        self.this_time_seconds = round(time.time(), 3)
        interval_ms = (self.this_time_seconds-self.last_time_seconds)

        #x_angular_raw = imu_data.angular_velocity.x*(interval_ms/1000)
        x_acc_velo = imu_data.linear_acceleration.x*(interval_ms)
        y_acc_velo = imu_data.linear_acceleration.y*(interval_ms)
        z_angular_velo = imu_data.angular_velocity.z
        
        knock_force = np.sqrt(np.square(x_acc_velo) + np.square(y_acc_velo))
        knock_force *= 1 if x_acc_velo < 0 else -1
        
        z_angular_velo += (knock_force*(KF_WEIGHT))
        
        self.last_angle += z_angular_velo*(interval_ms)
        
        if self.first_run:
            self.last_values = [self.last_angle for i in range(len(self.weights))]
            self.first_run = False
        
        self.last_values.pop(0)
        self.last_values.append(self.last_angle)
        return np.average(self.last_values, weights=self.weights)