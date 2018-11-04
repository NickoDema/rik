#!/usr/bin/env python

## python 2.7

##   ik_solver.py
##   Created on: 16.02.2018
##           By: Nikoaly Dema
##        Email: Nicko_Dema@protonmail.com

import os, sys
import rospy, tf, tf2_ros

from geometry_msgs.msg import Twist
from std_msgs.msg import String
#from tf2_msgs.msg import TFMessage

# add include directory to the system path
dpath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dpath + "/../include")
#print '\n'.join(sys.path)

from iiwa_modules import iiwa_kinematics

if __name__ == '__main__':
    iiwa = iiwa_kinematics.kinematic_server('iiwa')
    iiwa.hello()
    iiwa.spin()
