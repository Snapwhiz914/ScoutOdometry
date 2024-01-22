import numpy as np
import queue
from geometry_msgs.msg import Twist
from gryo_filter import GyroFilter
from tof_filter import TOFFilter
from collision_detection import CollisionDetector
import csv
import json
import time

from vo import VisualOdo

#Drive: Y+ is forward, Y- is backward, x+ is left, x- is right

TOF_CONV_FACTOR = 39.4
BARRIER_GAB_THRESH = 3
SLOPE_THRESH = 2
CURIOSITY_DIST = 25
MAP_DIST = 30
CURIOSITY_MOVE = 0.2
TOO_CLOSE_TO_ROTATE = 3.5


class Barrier():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.points = [(x1, y1), (x2, y2)]
        if (x1-x2) == 0.0: self.slope = None
        else: self.slope = (y1-y2)/(x1-x2)
    
    def is_point_on_line(self, x, y):
        if self.slope == None:
            if (
                abs(x-(self.x1-self.x2)) <= SLOPE_THRESH and
                self.y2 + BARRIER_GAB_THRESH > y and
                self.y1 - BARRIER_GAB_THRESH < y
            ):
                self.points.append((x, y))
                return True
            else: return False  
        if (
            abs((y - self.y1)-(self.slope*(x - self.x1))) <= SLOPE_THRESH and
            np.sqrt(np.power(abs(self.x1-x), 2) + np.power(abs(self.y1-y), 2)) < BARRIER_GAB_THRESH and
            np.sqrt(np.power(abs(self.x2-x), 2) + np.power(abs(self.y2-y), 2)) < BARRIER_GAB_THRESH
        ):
            self.points.append((x, y))
            return True
        return False

class Pathfinder():
    def __init__(self):
        self.angle_offset = None
        self.map = []
        self.pos = [0, 0]
        self.timestep = 0
        self.last_time_seconds = 0
        self.this_time_seconds = round(time.time(), 3)
        self.point_q = queue.Queue()
        self.curious_status = "forward"
        self.gyro_filter = GyroFilter()
        self.tof_filter = TOFFilter()
        self.col_detect = CollisionDetector()
        self.vo = VisualOdo("cam_profile.json")
        self.y_cmd = 0
        self.logga = csv.writer(open("log.csv", "w+", newline=''))

    def _process_tof(self, tof):
        filtered = self.tof_filter.loop(tof)
        return (filtered*TOF_CONV_FACTOR)
    
    def _process_imu(self, imu):
        a = np.rad2deg(self.gyro_filter.loop(imu))
        if self.angle_offset == None: self.angle_offset = a
        return a - self.angle_offset

    def _map(self, angle, distance):
        if distance > MAP_DIST: return
        # x_mult = 1
        # y_mult = 1
        # if angle > 0:
        #     if angle-90 >= 0: x_mult = -1
        #     elif angle-180 >= 0: x_mult, y_mult = -1, -1
        #     elif angle-270 >= 0: y_mult = -1
        # else:
        #     if angle+90 >= 0: y_mult = -1
        #     elif angle+180 >= 0: x_mult, y_mult = -1, -1
        #     elif angle+270 >= 0: x_mult = -1
        y = round(np.cos(np.deg2rad(angle))*distance, 1)
        x = round(np.sin(np.deg2rad(angle))*distance, 1)
        x+=self.pos[0]
        y+=self.pos[1]
        #Check if this point lies on another barrier, otherwise start a new barrier
        for barr in self.map:
            if barr.is_point_on_line(x, y):
                return
        #Start a new barrier
        if self.point_q.qsize() == 0:
            self.point_q.put((x, y))
        else:
            other_point = self.point_q.get()
            self.map.append(Barrier(x, y, other_point[0], other_point[1]))
    
    def _do_curiousity(self, angle, distance, collision):
        new_twist = Twist()
        if self.curious_status == "scan":
            if distance > CURIOSITY_DIST:
                print("Onward")
                self.curious_status = "forward"
                new_twist.linear.y = CURIOSITY_MOVE
                return new_twist
            else:
                if distance < TOO_CLOSE_TO_ROTATE:
                    new_y = 0.0
                    if collision:
                        new_twist.linear.y = (CURIOSITY_DIST/2)
                    else:
                        new_twist.linear.y = -(CURIOSITY_DIST/2)
                    y_move = np.cos(np.deg2rad(angle))*new_y
                    x_move = np.sin(np.deg2rad(angle))*new_y
                    self.pos[1] += y_move*(self.this_time_seconds-self.last_time_seconds)
                    self.pos[0] += x_move*(self.this_time_seconds-self.last_time_seconds)
                else:
                    new_twist.angular.z = 0.85
                return new_twist
        if distance < CURIOSITY_DIST:
            print("Scanning")
            self.curious_status = "scan"
            new_twist.angular.z = 0.85
            return new_twist
        if self.curious_status == "forward":
            y_move = np.cos(np.deg2rad(angle))*CURIOSITY_MOVE
            x_move = np.sin(np.deg2rad(angle))*CURIOSITY_MOVE
            self.pos[1] += y_move*(self.this_time_seconds-self.last_time_seconds)
            self.pos[0] += x_move*(self.this_time_seconds-self.last_time_seconds)
            #new_twist.linear.x = x_move
            #new_twist.linear.y = y_move
            new_twist.linear.y = CURIOSITY_MOVE
            return new_twist
    
    def process_input(self, imu, tof):
        self.last_time_seconds = self.this_time_seconds
        self.this_time_seconds = round(time.time(), 3)
        angle = self._process_imu(imu)
        distance = self._process_tof(tof)
        did_collide = self.col_detect.loop(imu.linear_acceleration, self.y_cmd)
        print(f"Angle = {angle}, Distance = {distance}")
        if did_collide: print("Did collide")
        self.logga.writerow((self.timestep, angle, distance))
        self.timestep+=1
        self._map(angle, distance)
        new_twist = self._do_curiousity(angle, distance, did_collide)
        self.y_cmd = new_twist.linear.y
        return new_twist

    def process_cam_input(self, cam_image):
        self.image_ts = self.timestep
        movement = self.vo.y_movement_since_last(cam_image)

    def save_map(self):
        barrier_list = [barr.points for barr in self.map]
        json.dump(barrier_list, fp=open("map_save.json", "w+"))