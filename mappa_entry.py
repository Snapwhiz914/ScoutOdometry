import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from pathfinder import Pathfinder

drive_pub = None
imu_data, tof_data = None, None

pathfindr = Pathfinder()

#TOF range: meters

def pathfind():
    global imu_data, tof_data
    drive_pub.publish(pathfindr.process_input(imu_data, tof_data))
    imu_data, tof_data = None, None

def imu_cb(data):
    global imu_data
    imu_data = data
    if imu_data != None and tof_data != None: pathfind()

def tof_cb(data):
    global tof_data
    tof_data = data
    if tof_data != None and imu_data != None: pathfind()

def img_cb(data):
    pathfindr.process_cam_input(data)
    #TODO: implement latency compensation

def main():
    global drive_pub
    
    #Publishers
    drive_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    #Subscribers
    imu_sub = rospy.Subscriber("/SensorNode/imu", Imu, imu_cb)
    tof_sub = rospy.Subscriber("/SensorNode/tof", Range, tof_cb)
    cam_sub = rospy.Subscriber("/CoreNode/grey_img")
    
    #Node
    rospy.init_node('pathfinder', anonymous=True, disable_signals=True)
    print("Node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
        pathfindr.save_map()
    except rospy.ROSInterruptException:
        pathfindr.save_map()
        print("Exiting")
        rospy.signal_shutdown("User exited")
        exit(0)