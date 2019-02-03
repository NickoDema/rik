#!/usr/bin/env python

## python 2.7

##   -------.py
##   Created on: 03.01.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com

import os, sys
import rospy, tf, tf2_ros

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

#from tf2_msgs.msg import TFMessage

import rik


if __name__ == '__main__':
    rospy.init_node("rik")
    robots = []

    descriptions = rospy.get_param('~descriptions', None)
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
        R = rik.Robot(name)
        if dtype == 'urdf':
            R.init_from_urdf(data)
        robots.append(rik.Robot())
    #Testy = rik.Robot(description)
    rospy.spin()
