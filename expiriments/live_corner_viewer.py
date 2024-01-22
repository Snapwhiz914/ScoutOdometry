import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
ql = 10

def on_change(pos):
    global ql
    divd = round(pos/10000, 6)
    if divd <= 0: divd = 0.001
    ql = divd

cv2.namedWindow("display")
cv2.createTrackbar('Quality Level', "display", 10, 5000, on_change)

def image_callback(msg):
    global bridge, ql
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg)
        gray_image = cv_image

        # Convert the image to grayscale
        #gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #print("converted")

        # Use goodFeaturesToTrack to find corners
        corners = cv2.goodFeaturesToTrack(gray_image, maxCorners=100, qualityLevel=ql, minDistance=10)

        # Convert corners to integers
        if str(type(corners)) == "<class 'NoneType'>": corners = []
        else: corners = np.int0(corners)

        # Draw corners on the image
        for corner in corners:
            x, y = corner.ravel()
            cv2.circle(cv_image, (x, y), 3, 255, -1)

        # Display the image with corners
        cv2.imshow("display", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing image: {}".format(str(e)))

def main():
    rospy.init_node('corner_detector_node', anonymous=True)

    # Subscribe to the "/CoreNode/greyimg" topic
    rospy.Subscriber("/CoreNode/grey_img", Image, image_callback)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
