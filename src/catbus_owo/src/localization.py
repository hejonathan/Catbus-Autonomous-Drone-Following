import rospy
import cv2
import numpy as np

class localization:

    def __init__(self):
        
        rospy.Subscriber("tello/video", Video)
        self.tello = Tello()
        self.tello.connect()

        # LOAD IMAGE AND TAG DICTIONARY
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        SIZE = 500

        # DETECT TAGS IN IMAGE
        marker = np.zeros((SIZE, SIZE, 1), dtype = np.uint8)
        
        # DRAW DETECTION AND SAVE FILE
        ID = 6
        cv2.aruco.drawMarker(arucoDict, ID, SIZE, marker, 1)
        cv2.imwrite('DICT_ARUCO_ORIGINAL_id_{}_{}.png'.format(ID, SIZE), marker)

    def detect(self):
        # LOAD IMAGE AND TAG DICTIONARY
        tags = cv2.imread('data/two_tags_ARUCO_ORIGINAL.png')
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        # DETECT TAGS IN IMAGE
        corners, ids, rejects = cv2.aruco.detectMarkers(cv2.cvtColor(tags, cv2.COLOR_BGR2GRAY), arucoDict)

        # DRAW DETECTION AND SAVE FILE
        detection = cv2.aruco.drawDetectedMarkers(tags, corners, borderColor=(255, 0, 0))
        cv2.imwrite('detection_two_tags_ARUCO_ORIGINAL.png', detection)
    
    def measure(self):
        self.tello.streamon()
        cv2.aruco.calibrateCameraAruco()
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=parameters)

        #actual tag size in meters
        markersize = 12.77

        rvec, tvec, _= cv2.aruco.estimatePoseSingleMarkers(corners, markersize, mtx, dist)

if __name__=='__main__':
    obj = localization()
