from roller_eye.msg import frame
import numpy as np
import rospy
import PIL.Image as im

sub: rospy.Subscriber = None
image = None
w, h = 0, 0
has_taken_pic = False

def save_img():
    global image
    print(len(image))
    imag = im.fromarray(np.array(image, dtype=np.uint8))
    imag.save("color_pic.jpg")

def cb(data: frame):
    global image, sub, has_taken_pic, w, h
    if has_taken_pic:
        save_img()
        sub.unregister()
        return
    print(f"{data.par1}, {data.par2}")
    w, h = data.par1, data.par2
    image = data.data
    has_taken_pic = True
    print("copied img")

sub = rospy.Subscriber("/CoreNode/h264", frame, cb)
rospy.init_node('e', anonymous=True, disable_signals=True)
rospy.spin()