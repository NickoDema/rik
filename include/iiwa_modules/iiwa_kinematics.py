#!/usr/bin/env python

## python 2.7

##   iiwa_kinematics.py
##   Created on: 16.02.2018
##           By: Nikoaly Dema
##        Email: Nicko_Dema@protonmail.com

import rospy
from math import pi

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import geometry_msgs.msg as g_msg
from tf.broadcaster import TransformBroadcaster
import tf.transformations as transforms
import tf

from terminaltables import SingleTable

class kinematic_server():
    def __init__(self, topic_name):
        rospy.init_node(topic_name, anonymous=True)
        # loading the Denavit-Hartenberg parameters table from ros
        # parameter server (joints limits parth is optional)
        self.DH_params_loaded = False
        self.j_limits_loaded = False
        self.DH = [['alpha', 'a', 'd', 'th']]#, 'th_low', 'th_up']]

        DH_full_param_name = rospy.search_param('iiwa_r820_DH')
        if (DH_full_param_name != None):
            params_table = rospy.get_param(DH_full_param_name)

            if (len(params_table[0]) == 4):
                rospy.logwarn('Joints limits is not set in parameters table')
            elif (len(params_table[0]) == 6):
                self.DH = [['alpha', 'a', 'd', 'th', 'th_low', 'th_up']]
                self.j_limits_loaded = True
            else:
                rospy.logerr('iiwa_r820_DH parameters table has a wrong format!')
                exit()

            self.DH.extend(params_table)
            self.DH_params_loaded = True

            # Generalized coordinates
            self.q = [0] * (len(self.DH))   #-1
            for i in range(1, len(self.DH)):
                self.DH[i][0] = self.DH[i][0]/180.0*pi
                #self.DH[i][0] = self.DH[i][3]/180*pi

            # print loaded table
            table = SingleTable(self.DH, 'DH')
            table.justify_columns = dict(zip(range(0, 6),['right' for x in range(6)]))
            print(table.table)
        else:
            rospy.logerr('iiwa_r820_DH parameters table has not been loaded!')
            exit()

        #def get_transform_FDH(self, to, from = 0):

    def spin(self):
        # create a timer to update the published transforms
        rospy.Timer(rospy.Duration(2.1), self.test_cb)
        rospy.spin()

    def test_cb(self, msg):
        self.hello()
        self.forward_DH()
        br = TransformBroadcaster()
        ls = tf.TransformListener()
        tar_2_msg = g_msg.TransformStamped()
        tar_3_msg = g_msg.TransformStamped()
        pos, ros = None, None

        try:
            ls.waitForTransform("tool_target", "iiwa_base_link", rospy.Time(0), rospy.Duration(4.0))
            (pos, rot) = ls.lookupTransform('iiwa_base_link', 'tool_target', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("can't get iiwa_base_link <- tool_target transform")
            return -1
        pos[0] += 0.4
        #br.sendTransform( pos, rot, rospy.Time.now(), 'tool_target_2', 'iiwa_base_link' )
        tar_2_msg.header.stamp = rospy.Time.now()
        tar_2_msg.header.frame_id = 'iiwa_base_link'
        tar_2_msg.child_frame_id = 'tool_target_2'
        tar_2_msg.transform.translation.x = pos[0]
        tar_2_msg.transform.translation.y = pos[1]
        tar_2_msg.transform.translation.z = pos[2]
        tar_2_msg.transform.rotation.x = rot[0]
        tar_2_msg.transform.rotation.y = rot[1]
        tar_2_msg.transform.rotation.z = rot[2]
        tar_2_msg.transform.rotation.w = rot[3]
        br.sendTransformMessage(tar_2_msg)

    def forward_DH(self, q=None, start=1, end=None):
        if q is None:
            q = self.q
        if end is None:
            end = len(self.q)

        q[4] = 0.4

        H = transforms.identity_matrix()
        for i in range(start, end):
            Rz = transforms.rotation_matrix(self.DH[i][3] + q[i], (0, 0, 1))
            Tz = transforms.translation_matrix((0, 0, self.DH[i][2]))
            Tx = transforms.translation_matrix((self.DH[i][1], 0, 0))
            Rx = transforms.rotation_matrix(self.DH[i][0], (1, 0, 0))
            A = transforms.concatenate_matrices(Rz, Tz, Tx, Rx)
            print(A)
            H = transforms.concatenate_matrices(H, A)
            #out = "%f\t%f\t%f\t%f + %f" % (self.DH[i][1], self.DH[i][0],
            #self.DH[i][2], self.q[i], self.DH[i][3])
        #rospy.logdebug(out)
        xyz = H[:3, 3]
        qtn = transforms.quaternion_from_matrix(H)
        rpy = transforms.euler_from_matrix(H[:3, :3], axes='sxyz')
        print(H)
        # check #################
        br = TransformBroadcaster()
        ls = tf.TransformListener()
        try:
            ls.waitForTransform("tool0", "iiwa_base_link", rospy.Time(0), rospy.Duration(4.0))
            (pos, rot) = ls.lookupTransform('iiwa_base_link', 'tool0', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("can't get iiwa_base_link <- tool0 transform")
            return -1
        print(pos)
        ##########################

        return xyz, qtn, rpy, H

    def hello(self):
        print('hello')
