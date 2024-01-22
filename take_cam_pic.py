from sensor_msgs.msg import Image
import numpy as np
import rospy
from cv_bridge import CvBridge
import cv2 as cv

sub: rospy.Subscriber = None
bridge = CvBridge()
image = None
w, h = 0, 0
has_taken_pic = False

def save_img():
    global image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    cv.imwrite("pic_.jpg", cv_image)

def cb(data: Image):
    global image, sub, has_taken_pic, w, h
    if has_taken_pic:
        save_img()
        sub.unregister()
        return
    print(f"{data.encoding}")
    print(f"{data.width}, {data.height}")
    w, h = data.width, data.height
    image = data
    has_taken_pic = True
    print("copied img")

sub = rospy.Subscriber("/CoreNode/grey_img", Image, cb)
rospy.init_node('e', anonymous=True, disable_signals=True)
rospy.spin()