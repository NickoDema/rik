#!/usr/bin/env python

## python 2.7

##   rik_test_node.py
##   Created on: 03.01.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com



from ros_wrapper import RikRosWrapper

if __name__ == '__main__':
    # to write driver for your robot redefine RikRosWraper methods
    rik_ros = RikRosWrapper()
    rik_ros.spin()
