import traceback
import rospy
import pygame
import numpy as np
from cv_bridge import CvBridge
from djitellopy import Tello
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Driver:
    def __init__(self) -> None:
        self.v = [0,0,0,0] #velocity in x,y,z,angular directions (this is initializing 'v')
        rospy.loginfo('Tello buddy says wasaaaaaah') #kickoff message
        rospy.init_node('driver', anonymous=True)
        self.vel_lis = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
        self.vel_request = rospy.Publisher('request_vel', Empty, queue_size=10)
        self.img_pub = rospy.Publisher('vid', Image, queue_size=10)
        self.MAX_SPEED = 100 #the tello may need to go fast sometimes
        self.down_cam = False #we want to start from the front camera
        self.FPS = 5 #choosing 5 frames per second to make image processing easier
        self.running = True
        self.tello = Tello() #Initializing Tello Tings
        self.tello.connect()
        self.tello.streamoff()
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read() #gets camera view
        self.br = CvBridge() #getting ready for later image conversion
        pygame.init() #initializing pygame in these 3 lines so that we can use key commands to start and stop the drone
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode((960, 720))
        
        
    def cmd_vel(self, data): #command velocity function provides velocity data in xyz as well as angular directions
        self.v = [data.linear.x, data.linear.y, data.linear.z, -data.angular.x]
        for i in range(len(self.v)):
            self.v[i] = min(self.v[i], self.MAX_SPEED)
            
    def start(self): #the start function is for taking off and setting running to True
        if not self.tello.is_flying:
            rospy.loginfo('Ready for takeoffffffff-')
            self.tello.takeoff()
        else:
            rospy.loginfo('My tello is flyyyyyyyyyying!')
        self.running = True
            
    def stop(self): #the stop function is for landing and setting running to False (used as an alternative exit in the run function)
        if self.tello.is_flying:
            rospy.loginfo('About to land!')
            self.tello.land()
        else:
            rospy.loginfo('On the ground already bub...')
        self.running = False
        
    def run(self): #The run function updates important commands/information in real time
        r = rospy.Rate(self.FPS)
        self.start()
        while self.running:
            for event in pygame.event.get(): #these lines provide the optino of pressing esc to stop everything
                if event.type == pygame.KEYDOWN:
                    rospy.loginfo(f'the master pressed the {str(event.key)}~~')
                    if event.key == pygame.K_ESACAPE:
                        self.stop()           
            frame = self.frame_read.frame #getting the most recent frame
            self.img_pub.publish(self.br.cv2_to_imgmsg(frame, 'bgr8')) #converting cv2 image to ross image form
            if self.tello.is_flying:
                    self.tello.send_rc_control(int(self.v[0]), int(self.v[1]), int(self.v[2]), int(self.v[3]))
            self.vel_request.publish()
                
            r.sleep() #add a delay in so everything computes eaasily
        self.stop()

if __name__ == '__main__':
    try:
        driver = Driver()  
        driver.run()
    except Exception as e: #the proper way to catch an error
        print(traceback.print_exc())
        print(str(e))
        print('RIP the code is failing (sent from Driver)')
        pass
