# Scout Odometry
Odometry pipeline for the Moorebot scout
 - Main file is `mappa_entry.py`
 - all 3d reconstruction/map building is done in `pathfinder.py`
 - all communication is done through ROS
 - Since the scout has a fisheye lense, the files in camera_calibration use the OpenCV camera calibration method
 - the expiriments folder has live views and the visual odometry pipeline expiriment
 - the logs folder has collections of `.csv` files of sensor data
