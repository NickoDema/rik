#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import geometry_msgs.msg
from tf.broadcaster import TransformBroadcaster
import tf

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
ls = None
tf_enbl = True
markers_pose = {}

def frameCallback( msg ):
    global br, ls, tf_enbl
    if tf_enbl:
        #t_tar = geometry_msgs.msg.TransformStamped()
        #t_err = geometry_msgs.msg.TransformStamped()

        # pos[0] += markers_pose["tool-target"].position.x
        # pos[1] += markers_pose["tool-target"].position.y
        # pos[2] += markers_pose["tool-target"].position.z
        # q = tf.transformations.quaternion_multiply(rot, (markers_pose["tool-target"].orientation.x,
        #                                                  markers_pose["tool-target"].orientation.y,
        #                                                  markers_pose["tool-target"].orientation.z,
        #                                                  markers_pose["tool-target"].orientation.w))
        mp = [markers_pose['tool-target'].position.x,
              markers_pose['tool-target'].position.y,
              markers_pose['tool-target'].position.z]
        mq = [markers_pose['tool-target'].orientation.x,
              markers_pose['tool-target'].orientation.y,
              markers_pose['tool-target'].orientation.z,
              markers_pose['tool-target'].orientation.w]

        # broadcast target frame  into TF
        br.sendTransform( mp, mq, rospy.Time.now(), 'tool_target', 'iiwa_base_link' )
        # t_tar.header.stamp = rospy.Time.now()
        # t_tar.header.frame_id = 'iiwa_base_link'
        # t_tar.child_frame_id = 'tool_target'
        # t_tar.transform.translation = mp
        # t_tar.transform.rotation = mq

        # if ls.frameExists("tool0") and ls.frameExists("iiwa_base_link"):
        # t = ls.getLatestCommonTime("tool0", "iiwa_base_link")
        # ls.waitForTransform("iiwa_base_link", "tool0",  rospy.Time(0), rospy.Duration(4.0))
        # try:
        #     (pos, rot) = ls.lookupTransform("iiwa_base_link", "tool0", rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print("can't get iiwa_base_link <- tool0 transform")
        #     return -1

        # print("mp : ",mp)
        # print("pos: ",pos)
        # print("mq : ",mq)
        # print("rot: ",rot)

        # mp[0] -= pos[0]
        # mp[1] -= pos[1]
        # mp[2] -= pos[2]
        # ls.waitForTransform("tool0", "iiwa_base_link", rospy.Time(0), rospy.Duration(4.0))
        # try:
        #     (pos, rot) = ls.lookupTransform("tool0", "iiwa_base_link", rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print("can't get iiwa_base_link <- tool0 transform1")
        #     return -1
        # mq = tf.transformations.quaternion_multiply(mq,rot)

        # broadcast an error between tool0 and target frames into TF
        #br.sendTransform( mp, mq, rospy.Time.now(), 'error', 'iiwa_base_link' )
        # t_err.header.stamp = rospy.Time.now()
        # t_err.header.frame_id = 'iiwa_base_link'
        # t_err.child_frame_id = 'error'
        # t_err.transform.translation = mp
        # t_err.transform.rotation = mq
        #
        # t = [t_tar, t_err]
        # br.sendTransform(t)


def move_to_frame_feedback( feedback ):
    int_marker = InteractiveMarker()
    int_marker = server.get(feedback.marker_name)
    global br, ls
    if (int_marker != None):
        try:
            (pos, rot) = ls.lookupTransform("iiwa_base_link", "tool0", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("can't get iiwa_base_link - tool0 transform")
            return -1
        int_marker.pose.position = Point( pos[0], pos[1], pos[2])
        int_marker.pose.orientation.w = rot[3]
        int_marker.pose.orientation.x = rot[0]
        int_marker.pose.orientation.y = rot[1]
        int_marker.pose.orientation.z = rot[2]
        server.insert(int_marker)
    server.applyChanges()

def tf_switcher( feedback ):
    global tf_enbl
    if (tf_enbl == True):
        tf_enbl = False
        print("TF OFF")
    else:
        tf_enbl = True
        print("TF ON")


def processFeedback( feedback ):

    # Feedback from marker 'simple_6dof_fixed_MENU' / control ''
    # : menu item 1 clicked at 0.00779414176941, 1.00928008556, 0.0681355595589 in frame base_link.

    # # s = "Feedback from marker '" + feedback.marker_name
    # s += "' / control '" + feedback.control_name + "'"
    #
    # mp = ""
    # if feedback.mouse_point_valid:
    #     mp = " at " + str(feedback.mouse_point.x)
    #     mp += ", " + str(feedback.mouse_point.y)
    #     mp += ", " + str(feedback.mouse_point.z)
    #     mp += " in frame " + feedback.header.frame_id
    #
    # if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
    #     rospy.loginfo( s + ": button click" + mp + "." )
    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
         rospy.loginfo(": menu item " + str(feedback.menu_entry_id) + " clicked" )
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        markers_pose["tool-target"] = feedback.pose
    #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    #    rospy.loginfo( s + ": mouse down" + mp + "." )
    #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
    #    rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.6
    marker.scale.y = msg.scale * 0.08
    marker.scale.z = msg.scale * 0.08
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 0.6
    marker.color.a = 1.0
    marker.pose.orientation.w = 0.707
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = -0.707
    marker.pose.orientation.z = 0.0
    marker.pose.position = Point( 0, 0, 0)
    return marker

def make_6dof_control( int_marker ):
    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.707
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #if fixed:
    #    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0.707
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    #if fixed:
    #    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0
    control.orientation.y = 0.707
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #if fixed:
    #    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0
    control.orientation.y = 0.707
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    #if fixed:
    #    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 0.707
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #if fixed:
    #    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 0.707
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 0.707
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    #if fixed:
    #    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)


def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


########################
# def make6DofMarker( fixed, interaction_mode, position, show_6dof = False):
# insert a box
# makeBoxControl(int_marker)

def make_moving_marker(frame, name, descript):
    global br, ls
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame
    int_marker.scale = 0.18
    int_marker.pose.orientation.w = 1.0
    int_marker.pose.orientation.x = 0.0
    int_marker.pose.orientation.y = 0.0
    int_marker.pose.orientation.z = 0.0
    int_marker.pose.position = Point( 0, 0, 0)
    ls.waitForTransform("tool0", "iiwa_base_link", rospy.Time(0), rospy.Duration(2.0))
    try:
       (pos, rot) = ls.lookupTransform("iiwa_base_link", "tool0", rospy.Time(0))
       int_marker.pose.position = Point( pos[0], pos[1], pos[2])
       int_marker.pose.orientation.w = rot[3]
       int_marker.pose.orientation.x = rot[0]
       int_marker.pose.orientation.y = rot[1]
       int_marker.pose.orientation.z = rot[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       print("can't get iiwa_base_link -> tool0 transform on start")
       return -1

    # try:
    #     (pos, rot) = ls.lookupTransform("iiwa_base_link", "tool0", rospy.Time(0))
    #     int_marker.pose.position = Point( pos[0], pos[1], pos[2])
    #     int_marker.pose.orientation.w = rot[3]
    #     int_marker.pose.orientation.x = rot[0]
    #     int_marker.pose.orientation.y = rot[1]
    #     int_marker.pose.orientation.z = rot[2]
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     print("something wrong")
    int_marker.name = name
    int_marker.description = descript

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(int_marker) )
    int_marker.controls.append(control)

    int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MENU

    make_6dof_control( int_marker )

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )
    markers_pose[name] = int_marker.pose
    print("+++++++++++++++++++++++++++++++++")
    print(markers_pose[name])
    print("+++++++++++++++++++++++++++++++++")


if __name__=="__main__":
    rospy.init_node("target_controller")
    br = TransformBroadcaster()
    ls = tf.TransformListener()

    server = InteractiveMarkerServer("target_controller")

    menu_handler.insert( "Move to frame", callback=move_to_frame_feedback )
    menu_handler.insert( "tf_ON/OFF", callback=tf_switcher )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )
    sub_menu_handle2 = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle2, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle2, callback=processFeedback )

    make_moving_marker( 'iiwa_base_link', 'tool-target', 'End-effector marker' )

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.02), frameCallback)

    server.applyChanges()

    rospy.spin()
