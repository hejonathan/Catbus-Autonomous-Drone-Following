from sympy import im
import rospy as rp
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty
import math
from collections import deque
import time
import numpy as np
from rviz_tools import RvisMarkers

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
        self.point_sub = rp.Subscriber('point', Twist, self.add_point) # subscribes from yolo, the location of the object
        self.vel_req_sub = rp.Subscriber('request_vel', Empty, self.publish_vel) # subscribe from driver, publishes velocity everytime this request is called
        self.location_sub = rp.Subscriber('drone_loc', Twist, self.add_drone_loc) # subscribe from localization, the location of the drone
        self.vel_pub = rp.Publisher('cmd_vel', Twist, queue_size=10) # publish the commanded velocity
        self.MIN_DIST = 10 # consecutive points that are within 10 cm of each other are ignored
        self.MAX_DIST = 100 # consecutive points that are further than 10 cm of each other are treated as error and ignored
        self.TOLERANCE_DIST = 25 # drone is considered reached the point if within 25 cm of the point
        self.markers = RvisMarkers('map', 'visualization_marker')
        rp.on_shutdown(self.markers.deteleAllMarkers())
        self.q = deque()
        self.rviz_drone = []
        self.rviz_obj = []
        self.RVIZ_MAX_LENGTH = 100
        self.pid = PID()
        self.drone_loc = np.array([0,0,0,0])
        rp.spin()
        
    def add_point(self, data):
        assert type(data) == Twist
        if len(self.q)==0 or self.MAX_DIST> self._dist(data, self.q[-1]) > self.MIN_DIST: # add to the list if the list is empty or the new point is far away enough from the last point
            self.q.append(data)
            rp.loginfo(f'owo~ added a point at {self.q[-1].linear.x} {self.q[-1].linear.y} {self.q[-1].linear.z}')
            
            self.rviz_obj.append(Point(x=data.linear.x, y=data.linear.y, z=data.linear.z))
            while len(self.rviz_obj) > self.RVIZ_MAX_LENGTH:
                self.rviz_obj.pop(0)
            self.markers.publishPath(self.rviz_obj, 'orange', 0,2)

    def publish_vel(self, data):
        if len(self.q) < 1: 
            rp.loginfo('aw so saddy saddy no points to track')
            return
        pt = self.q[0]
        err = np.array([pt.linear.x, pt.linear.y, pt.linear.z, 0]) - self.drone_loc
        self.pid.add_error(err)
        vel = self.pid.vel

        pub = Twist()
        pub.linear.x, pub.linear.z, pub.linear.y, pub.angular.x = vel
        self.vel_pub.publish(pub)

    def add__drone_loc(self, data):
        assert type(data) == Twist
        self.drone_loc = np.array([data.linear.x, data.linear.y, data.linear.z, data.angular.x]) # x, y, z, yaw of the drone
        while len(self.q) > 0 and self._dist(self.q[0], data) < self.TOLERANCE_DIST:
            self.q.popleft()
        
        self.rviz_drone.append(Point(x=data.linear.x, y=data.linear.y, z=data.linear.z))
        while len(self.rviz_drone) > self.RVIZ_MAX_LENGTH:
            self.rviz_drone.pop(0)
        self.markers.publishPath(self.rviz_drone, 'blue', 0.2)
        

    def _dist(a:Twist, b:Twist):
        dx = b.linear.x - a.linear.x
        dy = b.linear.y - a.linear.y
        dz = b.linear.z - a.linear.z
        return math.sqrt(dx**2 + dy**2 + dz**2)


if __name__=='__main__':
    obj = MotionControl()
