from sensor_msgs.msg import Image
import numpy as np
import rospy
from cv_bridge import CvBridge
import cv2 as cv
import os
import datetime
import curses

sub: rospy.Subscriber = None
bridge = CvBridge()
take_pic = False

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

if not os.path.exists("checkboard_pics"): os.mkdir("checkboard_pics")

def save_img(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    cv.imwrite(f"checkboard_pics/{datetime.datetime.now()}.jpg", cv_image)

def cb(data: Image):
    global sub, take_pic
    if take_pic:
        save_img(data)
        take_pic = False
        stdscr.addstr("picd\n")
        return

sub = rospy.Subscriber("/CoreNode/grey_img", Image, cb)
rospy.init_node('e', anonymous=True, disable_signals=True)

while True:
    k = stdscr.getkey()
    if k == 'q': break
    if k == 'p': take_pic = True

stdscr.addstr("done\n")