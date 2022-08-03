from sympy import im
import rospy as rp
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty
import math
from collections import deque
import time
import numpy as np
from rviz_tools import RvizMarkers

class PID:
    def __init__(self, ) -> None:
        '''
        [[px, ix, dx],
         [py, iy, dy],
         [pz, iz, dz],
         [pr, ir, dr]]
        '''
        x = np.array([.7, .3, .1]) * -1
        y = np.array([1.5, .05, .13]) * -1
        z = np.array([.8, .1, .1]) * 1
        r = np.array([1.5, .1, .02]) * 1
        self.constants = np.array([x, y, z, r]) # size: 4x3

        self.last_error = None
        self.last_time = None

        self.P, self.I, self.D = [np.zeros(4)]*3

        self.DIST = 100
        self.HEIGHT_OFFSET = 20
        
        self.vel = np.array([0,0,0,0])

    def add_error(self, err):
        current_time = time.time()
        self.last_time = current_time if self.last_time is None else self.last_time
        dt = current_time - self.last_time
        self.last_error = err if self.last_error is None else self.last_error

        self.P = err
        self.I += err * dt
        self.I = np.clip(self.I, -100, 100)
        if dt > 0:
            self.D = (err - self.last_error) / dt
        
        self.last_time = current_time
        self.last_error = err

        pid_arr = np.vstack((self.P, self.I, self.D)).T
        
        self.vel = np.sum(self.constants * pid_arr, axis=1).astype(int)

        rp.loginfo(f'error: {err}')


class MotionControl:
    def __init__(self) -> None:
        rp.init_node('motion_control', anonymous=True)
        self.point_sub = rp.Subscriber('displacement', Point, self.add_point) # subscribes from yolo, the location of the object
        self.vel_req_sub = rp.Subscriber('request_vel', Empty, self.publish_vel) # subscribe from driver, publishes velocity everytime this request is called
        self.location_sub = rp.Subscriber('cam_loc', Twist, self.add_drone_loc) # subscribe from localization, the location of the drone
        self.vel_pub = rp.Publisher('cmd_vel', Twist, queue_size=10) # publish the commanded velocity
        self.MIN_DIST = 10 # consecutive points that are within 10 cm of each other are ignored
        self.MAX_DIST = 100 # consecutive points that are further than 10 cm of each other are treated as error and ignored
        self.TOLERANCE_DIST = 25 # drone is considered reached the point if within 25 cm of the point
        rp.init_node('rviz', anomymous=False, log_level=rp.INFO, disable_signals=False)
        self.markers = RvizMarkers('map', 'visualization_marker')
        rp.on_shutdown(self.markers.deteleAllMarkers())
        self.q = deque()
        self.rviz_drone = [] # paths of the points tracking the locations of the drone and the head
        self.rviz_obj = []
        self.RVIZ_MAX_LENGTH = 100 # the maximum length of the above lists to prevent clogging up the memory
        self.pid = PID()
        self.drone_loc = np.array([0,0,0,0])
        rp.spin()
        
    def add_point(self, data):
        assert type(data) == Point
        if len(self.q)==0 or self.MAX_DIST> self._dist(data, self.q[-1]) > self.MIN_DIST: # add to the list if the list is empty or the new point is far away enough from the last point
            # transform from drone's fov to global fov
            # x: left right, y: forward back, z: up down
            angle = self.drone_loc[3] # define counter clockwise positive
            data.z += self.drone_loc[2]
            data.x = self.drone_loc[0] + data.x * math.cos(math.radians(angle)) + data.y * math.sin(math.radians(angle))
            data.y = self.drone_loc[1] + data.y * math.cos(math.radians(angle)) + data.x - math.sin(math.radians(angle))

            self.q.append(data)
            rp.loginfo(f'owo~ added a point at {self.q[-1].x} {self.q[-1].y} {self.q[-1].z}')
            
            self.rviz_obj.append(Point(x=data.x, y=data.y, z=data.z))
            while len(self.rviz_obj) > self.RVIZ_MAX_LENGTH:
                self.rviz_obj.pop(0)
            self.markers.publishPath(self.rviz_obj, 'orange', 0,2)

    def publish_vel(self, data):
        if len(self.q) < 1: 
            rp.loginfo('aw so saddy saddy no points to track')
            return
        pt = self.q[0]
        err = np.array([pt.x, pt.y, pt.z, 0]) - self.drone_loc # find the displacement between the drone and the point
        self.pid.add_error(err)
        vel = self.pid.vel # obtain the optimal velocity of the drone

        pub = Twist()
        pub.linear.x, pub.linear.y, pub.linear.z, pub.angular.x = vel
        self.vel_pub.publish(pub)

    def add__drone_loc(self, data):
        assert type(data) == Twist
        self.drone_loc = np.array([data.linear.x, data.linear.y, data.linear.z, data.angular.z]) # x, y, z, yaw of the drone
        while len(self.q) > 0 and self._dist(self.q[0], data) < self.TOLERANCE_DIST:
            self.q.popleft()
        
        self.rviz_drone.append(Point(x=data.linear.x, y=data.linear.y, z=data.linear.z))
        while len(self.rviz_drone) > self.RVIZ_MAX_LENGTH:
            self.rviz_drone.pop(0)
        self.markers.publishPath(self.rviz_drone, 'blue', 0.2)
        

    def _dist(a:Twist, b:Twist): # find distance between two points
        dx = b.linear.x - a.linear.x
        dy = b.linear.y - a.linear.y
        dz = b.linear.z - a.linear.z
        return math.sqrt(dx**2 + dy**2 + dz**2)


if __name__=='__main__':
    obj = MotionControl()
