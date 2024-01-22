#this vop uses the TOF sensor reading as a reference point
#meaning that it must start in an area where there is an accurate TOF reading
#then, move backwards slowly in order to collect keyframes
#use the change in feature position and the TOF reading change to obtain the proportion
#1 - ensure TOF reading, 2 - take inital picture, 3 - move backwards and keep taking pictures (say max 10), 4 - calculate proportion
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu, Image
from collision_detection import CollisionDetector
from tof_filter import TOFFilter
import time
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

w, h = 640, 480
fov_x, fov_y = np.deg2rad(78.5), np.deg2rad(50)
vpw, vph = 2.0*np.tan(fov_x/2), 2.0*np.tan(fov_y/2)

intrinsics_mtx = np.array([ [391.7276116, 0., 314.83665325],
                            [  0.,         524.27714316, 278.86406438],
                            [  0.,         0.,           1.,         ]
                        ])
dist_mtx = np.array([[-0.50668134,  0.30870431, -0.00488501,  0.00406649, -0.09989994]])
refinedmtx, roi = cv2.getOptimalNewCameraMatrix(intrinsics_mtx, dist_mtx, (w,h), 0, (w,h))
mapx,mapy=cv2.initUndistortRectifyMap(intrinsics_mtx,dist_mtx,None,refinedmtx,(w,h),5)

tof = TOFFilter()
col = CollisionDetector()
cv_bridge = CvBridge()
orb = cv2.ORB_create()
bf_matcher = cv2.BFMatcher()

current_tof = None
current_image = None
did_collide = False
current_twist = Twist()
take_pic = False
took_pic = False

#region Initialization
print("Initializing")

def pixel_to_xy_angle(x, y):
    nx, ny = ((1/(w/2)) * (x - (w/2))), ((1/(h/2)) * (y - (h/2)))
    return (
        np.atan2(1, (vpw/2)*nx),
        np.atan2(1, (vph/2)*ny)
    )

def tof_cb(range: Range): #TODO: ensure TOF none status is correct (refer to pathfinder cb)
    global current_tof
    current_tof = tof.loop(range)

def imu_cb(imu_data: Imu):
    global did_collide, current_twist
    if did_collide == False: did_collide = col.loop(imu_data.linear_acceleration, current_twist.linear.z)

def cam_cb(rospy_img: Image):
    global current_image, cv_bridge, take_pic, took_pic
    if take_pic and not took_pic:
        raw_img = cv_bridge.imgmsg_to_cv2(rospy_img, desired_encoding="passthrough")
        current_image = cv2.remap(raw_img,mapx,mapy,cv2.INTER_LINEAR)
        took_pic = True

def get_undistorted_image():
    global current_image, take_pic, took_pic
    take_pic = True
    while not took_pic: time.sleep(0.001)
    new_img = current_image
    take_pic = False; took_pic = False
    und_img = cv2.remap(new_img,mapx,mapy,cv2.INTER_LINEAR)
    return und_img

rospy.init_node("e", anonymous=True)
drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
tof_sub = rospy.Subscriber("/SensorNode/tof", tof_cb, queue_size=1)
imu_sub = rospy.Subscriber("/SensorNode/imu", imu_cb, queue_size=1)

#endregion
#region TOF calibration
print("Starting")
start_distance = int(input("How close to init object? > "))

current_twist.linear.y = 0.15
rate_10 = rospy.Rate(10)

while current_tof == None or current_tof > start_distance:
    drive_pub.publish(current_twist)
    rate_10.sleep()

current_twist.linear.y = 0
drive_pub.publish(current_twist)

time.sleep(0.75)
print("Ensured TOF reading, taking inital picture")
#endregion
#region Inital picture
inital_picture = get_undistorted_image()
initial_kp, initial_ds = orb.detectAndCompute(inital_picture,None)
#endregion
#region Backwards picture taking
current_twist.linear.y = -0.15

rate_5 = rospy.Rate(5)
query_pics = []
tofs = []
last_tof = tof
for i in range(10):
    if did_collide: break
    drive_pub.publish(current_twist)
    rate_5.sleep()
    query_pics.append(get_undistorted_image())
    tofs.append(tof-last_tof)
    last_tof = tof
did_collide = False

current_twist.linear.y = 0
drive_pub.publish(current_twist)

#endregion
#region Calculate prop
last_ds, last_kps = initial_ds, initial_kp
i = 0
props = []
for pic in query_pics:
    kps, ds = orb.detectAndCompute(pic, None)
    matches = bf_matcher.match(last_ds, ds)
    
    matches = sorted(matches,key=lambda x:x.distance)
    matches = matches[0:10]
    dists = []
    for match in matches:
        p1 = last_kps[match.queryIdx].pt
        p2 = kps[match.trainIdx].pt
        angle_p1, angle_p2 = pixel_to_xy_angle(p1.x, p1.y), pixel_to_xy_angle(p2.x, p2.y)
        angle_dist = np.sqrt(np.square(angle_p1[0]-angle_p2[0]) + np.square(angle_p1[1]-angle_p2[1]))
        dists.append(angle_dist)
    
    avg = np.average(dists)
    prop = avg/tofs[i]
    print(f"    {avg}")
    print(f"    {tofs[i]}")
    print(f" = {prop}")
    props.append(prop)
    
    last_kp = kps; last_ds = ds
    i+=1

print(np.average(props))
    
#endregion