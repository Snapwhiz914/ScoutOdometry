import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
w, h = 640, 480
intrinsics_mtx = np.array([ [391.7276116, 0., 314.83665325],
                            [  0.,         524.27714316, 278.86406438],
                            [  0.,         0.,           1.,         ]
                        ])
dist_mtx = np.array([[-0.50668134,  0.30870431, -0.00488501,  0.00406649, -0.09989994]])
refinedmtx, roi = cv2.getOptimalNewCameraMatrix(intrinsics_mtx, dist_mtx, (w,h), 0, (w,h))
mapx,mapy=cv2.initUndistortRectifyMap(intrinsics_mtx,dist_mtx,None,refinedmtx,(w,h),5)

cv2.namedWindow("og")
cv2.namedWindow("und")

def image_callback(msg):
    global bridge
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg)
        cv2.imshow("und", cv2.remap(cv_image,mapx,mapy,cv2.INTER_LINEAR))
        cv2.imshow("og", cv_image)
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
