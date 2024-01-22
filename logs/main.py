import csv
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

timesteps = []
angle = []

with open("log_mrun2.csv", 'r') as file:
    csv_reader = csv.reader(file)
    # next(csv_reader)
    # next(csv_reader)
    
    for row in csv_reader:
        timesteps.append(float(row[0]))
        angle.append(float(row[1]))

ekf = ExtendedKalmanFilter(dim_x=1, dim_z=1)
ekf.x = angle[2]
ekf.F = 1

plt.plot(timesteps, angle, marker='o', linestyle='-', color='b')
#plt.plot(timesteps, , linestyle='-', color='r')
plt.xlabel('Timestep')
plt.ylabel('Angle (deg)')
plt.grid(True)
plt.show()

