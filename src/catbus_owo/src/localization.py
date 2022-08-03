#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

MARKER_LENGTH = 13.3
# MARKER_LENGTH = 6
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
R_FLIP = np.array([[1,0,0],[0,-1,0],[0,0,-1]])

class Localization:
    def __init__(self) -> None:
        rospy.init_node('localizatoin', anonymous=True)
        self.img_sub = rospy.Subscriber('vid', Image, self.callback)
        self.img_lbl_pub = rospy.Publisher('vid_lbl', Image, queue_size=10)
        self.tag_loc_pub = rospy.Publisher('tag_loc', Twist, queue_size=10)
        self.cam_loc_pub = rospy.Publisher('cam_loc', Twist, queue_size=10)
        self.found_pub = rospy.Publisher('tag_found', Bool, queue_size=10)
        self.found = False
        self.br = CvBridge()
        self.FPS = 30
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        rospy.spin()

    def callback(self, data):
        img = self.br.imgmsg_to_cv2(data,'bgr8')
        if img is None:
            return
        corners, ids, rejects = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), self.arucoDict)
        if ids is not None and len(ids) > 0:
            corners = tuple([np.array(corners)[np.where(ids==7)]])
            ids = tuple([np.array(ids)[np.where(ids==7)]])
            if corners[0].shape[0] == 0:
                return
            self.found = True
                
            img_lbl = cv2.aruco.drawDetectedMarkers(img, corners)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix, distortion) # For a single marker
            (rvec-tvec).any()
            
            img_lbl = cv2.drawFrameAxes(img_lbl, camera_matrix, distortion, rvec[0], tvec[0], 5)    # axis length 100 can be changed according to your requirement
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T
            tag_pitch, tag_yaw, tag_roll = self._rotationMatrix2EulerAngles(R_tc)
            tag_pitch += 180
            tag_loc = Twist()
            tag_loc.linear.x, tag_loc.linear.z, tag_loc.linear.y = tvec[0][0]
            tag_loc.angular.x, tag_loc.angular.y, tag_loc.angular.z = tag_roll, tag_pitch, tag_yaw
            self.tag_loc_pub.publish(tag_loc)

            cam_loc = Twist()
            cam_loc.linear.x, cam_loc.linear.z, cam_loc.linear.y = (-R_tc*np.matrix(tvec).T).astype(int)
            cam_roll, cam_pitch, cam_yaw = self._rotationMatrix2EulerAngles(R_FLIP*R_tc).astype(int)
            cam_loc.angular.x, cam_loc.angular.y, cam_loc.angular.z = cam_roll, cam_pitch, cam_yaw
            self.cam_loc_pub.publish(cam_loc)
        else:
            img_lbl = img
        self.found_pub.publish(Bool(self.found))
        self.img_lbl_pub.publish(self.br.cv2_to_imgmsg(img_lbl, 'bgr8'))
            
    def _rotationMatrix2EulerAngles(self, R):
        sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
        singular = sy<1e-6
        if not singular:
            x = math.atan2(R[2,1], R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.rad2deg(np.array([x,y,z]))

if __name__ == '__main__':
    detect = Localization()