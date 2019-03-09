#!/usr/bin/env python

##   kinematics.py
##   Created on: 04.02.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##
##   Module discription TODO!

## python 2.7

import os, sys
import rospy, tf, tf2_ros

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

#from tf2_msgs.msg import TFMessage

import rik

class RikRosWrapper():
    def __init__(self, description_param_name = '~descriptions'):

        rospy.init_node("rik")
        robots = []

        rospy.get_param('log', False)

        descriptions = rospy.get_param(description_param_name, None)
        if descriptions is None:
            rospy.logerr('[RIK]: There is no \"descriptions\" parameter')
            exit()

        for description in descriptions:
            name =  description['name']
            dtype = description['dtype']
            data = rospy.get_param(description['data'], None)
            if data is None:
                rospy.logwarn('[RIK]: There is no description for ', name, ' ')
                continue
            R = rik.Robot(name, True)
            if dtype == 'urdf':
                R.init_from_urdf(data)
                robots.append(R)
            else:
                rospy.logwarn('[RIK]: Solver does\'n support ', dtype,
                              'description type (for ', name, ' robot)')
                continue


    def spin(self):
        rospy.spin()
