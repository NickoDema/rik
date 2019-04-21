#!/usr/bin/env python

##   kinematics.py
##   Created on: 03.01.2019
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##
##   Within Reaching Inverse Kinematics algorithm the full robot kinematic
## chain is represented as a directed graph.
## TODO Full description

## python 2.7

from math import pi

from rik.parsers import urdf_to_model

# https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py
# first character : rotations are applied to 's'tatic or 'r'otating frame
import tf.transformations as tf

# just for pretty printing of parsed model
import pprint
#from terminaltables import SingleTable


# class Transformation():
#     class Translation():
#         def __init__(self):
#             self.x = 0
#             self.y = 0
#             self.z = 0
#
#     class Rotation():
#         def __init__(self):
#             self.x = 0
#             self.y = 0
#             self.z = 0
#             self.w = 1


def fk(state):
    pass

def pos(state, targets):
    pass

def vel(state, targets):
    pass

def acc(state, targets):
    pass


#   Class Joint stores current joint coordinate, velocity, acceleration
# in a configuration space, full parent -> child transformation, joint
# type, name, etc
#
#   Full transformation H between parent and child links is generally
# presented as:
#
#    H = Hb * Hj * Ha
#
# Special cases: For URDF Ha is equal to I.
#                For Denavit-Hartenberg (DH) convention Hb is equal to I
class Joint():

    # Joint limits
    class Limits():
        def __init__(self):
            self.low = None
            self.upp = None
            self.vel = None
            self.acc = None
            self.dec = None
            self.eff = None

    def __init__(self):
        self.name   = ""
        self.type   = ""          # p - prismatic | r - revolute
                                  # fl - floating | f - fixed

        self.parent_link = None
        self.child_link  = None

        self.Hb = None          # Transformation before J
        self.Hj = None          # Transformation by J
        self.Ha = None          # Transformation after J | equal to I in urdf

        self.pos = 0
        self.vel = 0
        self.acc = 0
        self.lim = self.Limits()


#   All pose, velocity, acceleration values for Target and Node classes are
# expressed relative to the base coordinate system.
class Target():
    def __init__(self):
        self.name = ""
        self.priority = 1               # priority
        self.pos = None
        self.vel = None
        self.acc = None

    def set_priority(self, priority):
        self.priority = priority

#   If the frame has no parent in a description (urdf for example) it will be
# initialized as a fixed
class Node():

    def __init__(self):
        self.name = ""
        self.type = 0                   # 1 for fixed | 0 for free
        self.target = None
        self.branch = None
        self.pos = None
        self.vel = None
        self.acc = None
        self.parent_frames = []
        self.child_frames  = []

    def set_fixed(self):
        self.type = 1

    def set_free(self):
        self.type = 0

    def set_target(self):
        pass

    def branch(self):
        return self.branch

#   Branch object is a set consisted of nodes. Branch starts and ends by nodes
# with more then one parents or childs or by nodes which has no parent or child.
#   Branches with common node is siblings.
class Branch():
    def __init__(self):
        self.nodes = {}
        self.siblings = {}

    def has_target(self):
        pass

    #   Return Loop objects in which this branch is used. None otherwise.
    def in_loop(self):
        pass

#   If two or more siblings has common child frame they form a Loop.
class Loop():
    def __init__(self):
        pass

# Class Robot stores full robot state
class Robot():

    def __init__(self, name = None, log = False):
        self.name = name
        self.inited = False
        # self.joint_num = 0
        self.frame_num = 0

        self.joints  = {}
        self.nodes   = {}
        self.targets = {}                 # {"name" : ik_order, .. }

        self.log = log
        self.pp = pprint.PrettyPrinter(indent=2)

    def add_node(self, node):
        self.nodes.update({node.name: node})

    def get_node(self, name):
        if name in self.nodes:
            return self.nodes[name]
        else:
            return None

    def add_joint(self, joint):
        self.joints.update({joint.name: joint})

    def set_joints(self, joints_state):
        pass

    def add_target(self, target):
        pass

    #   URDF uses a tree structure to represent robot kinematics, which does
    # not permit closed loops
    def init_from_urdf(self, urdf):

        robot_model = urdf_to_model(urdf)

        if self.log: self.pp.pprint(robot_model)

        links = robot_model['links']
        joints = robot_model['joints']

        # TODO errors!

        for link_name in links:
            node = Node()
            node.name = link_name
            self.add_node(node)
        #print(self.nodes)

        for joint_name in joints:
            joint = Joint()
            joint.name = joint_name
            joint_data = joints[joint_name]
            joint_origin = joint_data['origin']

            joint.parent_link = self.get_node(joint_data['parent'])
            if self.log and joint.parent_link is None:
                print("[RIK]: ", joint_name, " has no parent link")

            joint.child_link  = self.get_node(joint_data['child'])
            if self.log and joint.child_link is None:
                print("[RIK]: ", joint_name, " has no child link")

            Rb = tf.euler_matrix(joint_origin['roll'], joint_origin['pitch'],
                                 joint_origin['yaw'], 'rxyz')

            Pb = tf.translation_matrix((joint_origin['x'], joint_origin['y'],
                                        joint_origin['z']))

            Hb = tf.concatenate_matrices(Pb, Rb)
            joint.Hb = Hb

            if joint_data['type'] == 'fixed':
                joint.type = "f"

            if joint_data['type'] == 'revoute' or joint_data['type'] == 'continuous':
                joint.type = "r"

                if 'zero_state' in joint_data:
                    joint.pos = joint_data['zero_state']

                joint_axis = joint_data['axis']

                joint.Hj = None          # Transformation by J
                joint.Ha = None          # Transformation after J | equal to I in urdf



                self.pos = 0
                self.vel = 0
                self.acc = 0
                self.lim = self.Limits()

            if joint_data['type'] == 'prismatic':
                joint.type = "p"


            self.add_joint(joint)

        if self.log: self.pp.pprint(self.nodes)
        if self.log: self.pp.pprint(self.joints)

    def init_from_urdf2(self, urdf2):
        print(urdf2)

    def init_from_sdf(self, sdf):
        print(sdf)

    def init_from_DH(self, dh):
        print(dh)

    def spin(self):
        pass



# # Base Robot class
# class Robot():
#     def __init__(self, description='None'):
#         #rospy.init_node(node_name, anonymous=True)
#
#         # loading the Denavit-Hartenberg parameters table from ros
#         # parameter server (joints limits parth is optional)
#     #     self.DH_params_loaded = False
#     #     self.j_limits_loaded = False
#     #     self.DH = [['alpha', 'a', 'd', 'th']]#, 'th_low', 'th_up']]
#     #
#     #     DH_full_param_name = rospy.search_param('iiwa_r820_DH')
#     #     if (DH_full_param_name != None):
#     #         params_table = rospy.get_param(DH_full_param_name)
#     #
#     #         if (len(params_table[0]) == 4):
#     #             rospy.logwarn('Joints limits is not set in parameters table')
#     #         elif (len(params_table[0]) == 6):
#     #             self.DH = [['alpha', 'a', 'd', 'th', 'th_low', 'th_up']]
#     #             self.j_limits_loaded = True
#     #         else:
#     #             rospy.logerr('iiwa_r820_DH parameters table has a wrong format!')
#     #             exit()
#     #
#     #         self.DH.extend(params_table)
#     #         self.DH_params_loaded = True
#     #
#     #         # Generalized coordinates
#     #         self.q = [0] * (len(self.DH))   #-1
#     #         for i in range(1, len(self.DH)):
#     #             self.DH[i][0] = self.DH[i][0]/180.0*pi
#     #             #self.DH[i][0] = self.DH[i][3]/180*pi
#     #
#     #         # print loaded table
#     #         table = SingleTable(self.DH, 'DH')
#     #         table.justify_columns = dict(zip(range(0, 6),['right' for x in range(6)]))
#     #         print(table.table)
#     #     else:
#     #         rospy.logerr('iiwa_r820_DH parameters table has not been loaded!')
#     #         exit()
#     #
#     #     #def get_transform_FDH(self, to, from = 0):
#     #
#     # def spin(self):
#     #     # create a timer to update the published transforms
#     #     rospy.Timer(rospy.Duration(2.1), self.test_cb)
#     #     rospy.spin()
#     #
#     # def test_cb(self, msg):
#     #     self.hello()
#     #     self.forward_DH()
#     #     br = TransformBroadcaster()
#     #     ls = tf.TransformListener()
#     #     tar_2_msg = g_msg.TransformStamped()
#     #     tar_3_msg = g_msg.TransformStamped()
#     #     pos, ros = None, None
#     #
#     #     try:
#     #         ls.waitForTransform("tool_target", "iiwa_base_link", rospy.Time(0), rospy.Duration(4.0))
#     #         (pos, rot) = ls.lookupTransform('iiwa_base_link', 'tool_target', rospy.Time())
#     #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#     #         print("can't get iiwa_base_link <- tool_target transform")
#     #         return -1
#     #     pos[0] += 0.4
#     #     #br.sendTransform( pos, rot, rospy.Time.now(), 'tool_target_2', 'iiwa_base_link' )
#     #     tar_2_msg.header.stamp = rospy.Time.now()
#     #     tar_2_msg.header.frame_id = 'iiwa_base_link'
#     #     tar_2_msg.child_frame_id = 'tool_target_2'
#     #     tar_2_msg.transform.translation.x = pos[0]
#     #     tar_2_msg.transform.translation.y = pos[1]
#     #     tar_2_msg.transform.translation.z = pos[2]
#     #     tar_2_msg.transform.rotation.x = rot[0]
#     #     tar_2_msg.transform.rotation.y = rot[1]
#     #     tar_2_msg.transform.rotation.z = rot[2]
#     #     tar_2_msg.transform.rotation.w = rot[3]
#     #     br.sendTransformMessage(tar_2_msg)
#     #
#     # def forward_DH(self, q=None, start=1, end=None):
#     #     if q is None:
#     #         q = self.q
#     #     if end is None:
#     #         end = len(self.q)
#     #
#     #     q[4] = 0.4
#     #
#     #     H = transforms.identity_matrix()
#     #     for i in range(start, end):
#     #         Rz = transforms.rotation_matrix(self.DH[i][3] + q[i], (0, 0, 1))
#     #         Tz = transforms.translation_matrix((0, 0, self.DH[i][2]))
#     #         Tx = transforms.translation_matrix((self.DH[i][1], 0, 0))
#     #         Rx = transforms.rotation_matrix(self.DH[i][0], (1, 0, 0))
#     #         A = transforms.concatenate_matrices(Rz, Tz, Tx, Rx)
#     #         print(A)
#     #         H = transforms.concatenate_matrices(H, A)
#     #         #out = "%f\t%f\t%f\t%f + %f" % (self.DH[i][1], self.DH[i][0],
#     #         #self.DH[i][2], self.q[i], self.DH[i][3])
#     #     #rospy.logdebug(out)
#     #     xyz = H[:3, 3]
#     #     qtn = transforms.quaternion_from_matrix(H)
#     #     rpy = transforms.euler_from_matrix(H[:3, :3], axes='sxyz')
#     #     print(H)
#     #     # check #################
#     #     br = TransformBroadcaster()
#     #     ls = tf.TransformListener()
#     #     try:
#     #         ls.waitForTransform("tool0", "iiwa_base_link", rospy.Time(0), rospy.Duration(4.0))
#     #         (pos, rot) = ls.lookupTransform('iiwa_base_link', 'tool0', rospy.Time())
#     #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#     #         print("can't get iiwa_base_link <- tool0 transform")
#     #         return -1
#     #     print(pos)
#     #     ##########################
#     #
#     #     return xyz, qtn, rpy, H
#
#     def spin(self):
