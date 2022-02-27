import can
from math import *
import time



class robot(object):
    def __init__(self, wheel_radius, track_width):
        self._wheel_radius = wheel_radius
        self._track_width = (1/2)*track_width           # Distance betweer right and left wheel, in this implementation we take half of it
        self.odom = {'x':0,'y':0,'theta':0,'v':0,'w':0}

    # This calculates the left and right wheel speeds in rad/s from vx and w        
    def calWheelVel(self,v,w):
        wr = 1/self._wheel_radius *(v + w * self._track_width)
        wl = 1/self._wheel_radius *(v - w * self._track_width)
        return {wl,wr}

    # This calculates vx and w from the left and right wheel speeds in rad/s  
    def calRobotOdom(self, wr, wl):
        dt = 0.05
        w = self._wheel_radius/self._track_width * (wr - wl)
        v = self._wheel_radius/2*(wr + wl)
        self.odom['v'] = v
        self.odom['w'] = w






