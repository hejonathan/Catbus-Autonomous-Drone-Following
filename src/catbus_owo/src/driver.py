import traceback
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import rospy
import cv2
import math
import pygame
import numpy as np
from cv_bridge import CvBridge
from djitellopy import Tello
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Driver:
    def __init__(self) -> None:
        self.vel_lis = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
        self.img_pub = rospy.Publisher('vid', Image, queue_size=10)
        self.MAX_SPEED = 80
        self.down_cam = False
        self.FPS = 30
        self.running = True
        self.MAX_SPEED = 80
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamoff()
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read() #gets camera view
        self.br = CvBridge() #getting ready for later image conversion
        pygame.init()
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode((960, 720))
        
        
    def cmd_vel(self, data):
        self.v = [data.linear.x, data.linear.y, data.linear.z, -data.angular.x]
        for i in range(len(self.v)):
            self.v[i] = min(self.v[i], self.MAX_SPEED)
            
    def start(self, data):
        if not self.tello.is_flying:
            rospy.loginfo('Ready for takeoffffffff-')
            self.tello.takeoff()
        else:
            rospy.loginfo('My tello is flyyyyyyyyyying!')
        #tello starts flying here
            
    def stop(self, data):
        if self.tello.is_flying:
            rospy.loginfo('About to land!')
            self.tello.land()
        else:
            rospy.loginfo('On the ground already bub...')
        self.running = False
        
    def run(self):
        r = rospy.Rate(self.FPS)
        self.running = True
        while self.running:
            self.start()
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    rospy.loginfo(f'the master pressed the {str(event.key)}~~')
                    if event.key == pygame.K_ESACAPE:
                        self.stop()           
            frame = self.frame_read.frame
            self.img_pub.publish(self.br.cv2_to_imgmsg(frame, 'bgr8')) #converting cv2 image to ross image form
            if self.tello.is_flying:
                    self.tello.send_rc_control(int(self.v[0]), int(self.v[1]), int(self.v[2]), int(self.v[3]))
            r.sleep()
        self.stop()

if __name__ == '__main__':
    try:
        driver = Driver()  
        driver.run()
    except Exception as e:
        print(traceback.print_exc())
        print(str(e))
        print('RIP the code is failing (sent from Driver)')
        pass


#RVIZ:


#rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
#def cleanup_node():
#     print "Shutting down node"
#     markers.deleteAllMarkers()
# rospy.on_shutdown(cleanup_node)
# markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')

# Hpath = 
# Dpath = 
# go through and draw line's between every point and publish to markers

# rospy.Rate(1).sleep() #1 Hz

# display the plot markers