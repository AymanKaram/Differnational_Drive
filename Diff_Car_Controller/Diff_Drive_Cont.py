"""
BSD 3-Clause License
Copyright (c) 2022, Mohamed Abdelkader Zahana
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
@brief This is a differntioanl drive robot controller.
"""

import logging
from tkinter import Y
import  ZLAC8030L_CAN_controller.canopen_controller
from ZLAC8030L_CAN_controller.canopen_controller import MotorController

from math import *
import time



class robot(object):
    def __init__(self, wheel_radius, track_width, can_network):

        self._can_net = can_network
        self._wheel_radius = wheel_radius
        self._track_width = (1/2)*track_width           # Distance betweer right and left wheel, in this implementation we take half of it
        self.odom = {'x':0,'y':0,'yaw':0,'x_dot':0,'y_dot':0,'v':0,'w':0}
        

    # This calculates the left and right wheel speeds in rad/s from vx and w        
    def calWheelVel(self,v,w):
        wr = 1/self._wheel_radius *(v + w * self._track_width)
        wl = 1/self._wheel_radius *(v - w * self._track_width)
        return [wl,wr]

    # This calculates vx and w from the left and right wheel speeds in rad/s  
    def calRobotOdom(self, dt):
        # Calculate the virtual left and right wheel angular position and velocity
        a1 = self._can_net.getEncoder(1) # These are the actual wheel positions for the robot starting from the front left 
        a2 = self._can_net.getEncoder(2) # These are the actual wheel positions for the robot starting from the rear left 
        a3 = self._can_net.getEncoder(3) # These are the actual wheel positions for the robot starting from the rear right
        a4 = self._can_net.getEncoder(4) # These are the actual wheel positions for the robot starting from the front right  

        w1 = self._can_net.getVelocity(1) # These are the current wheel velocity for the robot starting from the front left
        w2 = self._can_net.getVelocity(2) # These are the current wheel velocity for the robot starting from the rear left
        w3 = self._can_net.getVelocity(3) # These are the current wheel velocity for the robot starting from the rear right
        w4 = self._can_net.getVelocity(4) # These are the current wheel velocity for the robot starting from the front right

        angular_pos_l = (a1 + a2)/2
        angular_pos_r = (a3 + a4)/2

        wl = (w1 + w2)/2
        wr = (w3 + w4)/2

        angular_pos = (angular_pos_r - angular_pos_l)*self._wheel_radius/(2*self._track_width)



        angular_vel = self._wheel_radius/(2*self._track_width) * (wr - wl)
        linear_vel = (self._wheel_radius/2)*(wr + wl)


        x_dot = linear_vel * cos(angular_pos)
        y_dot = linear_vel * sin(angular_pos)

        self.odom['x']= self.odom['x'] + dt* x_dot
        self.odom['y']= self.odom['y'] + dt* y_dot

        self.odom['x_dot'] = x_dot
        self.odom['y_dot'] = y_dot

        self.odom['v'] = linear_vel
        self.odom['w'] = angular_vel
        self.odom['yaw'] = angular_pos


def main():
    print("This is odom and Kinematics implementation of Diff_Drive_Cont.py script. \n")
    network_obj = MotorController(channel='can0', bustype='socketcan_ctypes', bitrate=500000, node_ids=None, debug=True, eds_file='./eds/ZLAC8030L-V1.0.eds')

    ugv = robot(wheel_radius= 0.194, track_width= 0.6405, can_network = network_obj)



#    # Get some velocities
#    t1 = time.time()
#    N = 1000
#    for i in range(N):
#      vel =  obj.getVelocity(node_id)
#    #   logging.info("Curent velocity = {} rpm \n".format(vel))

#      obj.setVelocity(node_id=node_id, vel=i/10.)




if __name__=="__main__":
    main()


