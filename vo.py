import numpy as np
from cv_bridge import CvBridge
import cv2
import json

class VisualOdo:
    def __init__(self, cam_profile_path):
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create()
        self.bf_matcher = cv2.BFMatcher()
        self.cam_prof = self._decode_cam_prof(cam_profile_path)
        self.wh = (self.cam_prof["width"], self.cam_prof["height"])
        self.definedmtx, self.roi = cv2.getOptimalNewCameraMatrix(self.cam_prof["intrinsics"], self.cam_prof["distortion"], self.wh, 0, self.wh)
        self.mapx,self.mapy=cv2.initUndistortRectifyMap(self.cam_prof["intrinsics"],self.cam_prof["distortion"],None,self.definedmtx,self.wh,5)
        self.vpw, self.vph = 2.0*np.tan(self.cam_prof["fov_x"]/2), 2.0*np.tan(self.cam_prof["fov_y"]/2)
        self.last_pic, self.last_ds, self.last_kps = None, None, None
    
    def _decode_cam_prof(self, path):
        f = open(path, "r")
        raw = json.load(f)
        f.close()
        raw["intrinsics"] = np.array(raw["intrinsics"])
        raw["distortion"] = np.array(raw["distortion"])
        raw["x_fov"] = np.deg2rad(raw["x_fov"])
        raw["y_fov"] = np.deg2rad(raw["y_fov"])
        return raw

    def _pixel_to_xy_angle(self, x, y):
        nx, ny = ((1/(self.wh[0]/2)) * (x - (self.wh[0]/2))), ((1/(self.wh[1]/2)) * (y - (self.wh[1]/2)))
        return (
            np.arctan2(1, (self.vpw/2)*nx),
            np.arctan2(1, (self.vph/2)*ny)
        )

    def y_movement_since_last(self, img_msg):
        raw_img = self.bridge.imgmsg_to_cv2(img_msg)
        undistorted = cv2.remap(raw_img,self.mapx,self.mapy,cv2.INTER_LINEAR)
        kps, ds = self.orb.detectAndCompute(undistorted, None)
        if self.last_pic == None:
            self.last_pic = undistorted
            self.last_ds = ds
            self.last_kps = kps
            return 0
        matches = self.bf_matcher.match(self.last_ds, ds)
        
        matches = sorted(matches,key=lambda x:x.distance)
        dists = []
        matches = matches[:50]
        for match in matches:
            p1 = self.last_kps[match.queryIdx].pt
            p2 = kps[match.trainIdx].pt
            angle_p1, angle_p2 = self._pixel_to_xy_angle(p1[0], p1[1]), self._pixel_to_xy_angle(p2[0], p2[1])
            angle_dist = np.sqrt(np.square(angle_p1[0]-angle_p2[0]) + np.square(angle_p1[1]-angle_p2[1]))
            dists.append(angle_dist)
    
        avg = np.average(dists)
        return avg/self.cam_prof["distance_prop"]