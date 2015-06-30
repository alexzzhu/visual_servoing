#!/usr/bin/env python
import baxter
from baxter_pykdl import baxter_kinematics
import baxter_interface

import numpy as np
import roslib
import rospy
import sys
from tf.transformations import quaternion_matrix
import tf
from utility import *
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

class BaxterVS(object):
    def __init__(self,limb):
        baxter.open_right_arm_cam_small()

        transform=baxter.get_tf_listener()

        transform.waitForTransform('/' + limb + '_hand','/' + limb + '_hand_camera',rospy.Time(0),rospy.Duration(5.0))
        (self._cam2hand_t,self._cam2hand_R)=transform.lookupTransform('/' + limb + '_hand','/' + limb + '_hand_camera',rospy.Time(0))
        self._cam2hand_t= np.concatenate((np.transpose(np.matrix(self._cam2hand_t)),np.matrix([[0]])),axis=0)
        self._cam2hand_R=quaternion_matrix(self._cam2hand_R)
        self._arm=baxter_interface.limb.Limb(limb)
        self._kin = baxter_kinematics(limb)

    def cam_to_body(self, vector):
        cam2hand = generate_frame_transform(self._cam2hand_t[0:3,:],self._cam2hand_R[0:3,0:3],False)
        #hand_pose = baxter.get_arm_camera_pose(baxter.RIGHT)
        hand_pose = baxter.get_arm_pose(baxter.RIGHT)
        (t,R) = get_t_R(hand_pose)
        hand2body = generate_frame_transform(t[0:3,:],R[0:3,0:3],True)
        return np.dot(hand2body,np.dot(cam2hand,vector))

    def set_hand_vel(self,vel):
         # Calculate joint velocities to achieve desired velocity
        joint_vels=np.dot(self._kin.jacobian_pseudo_inverse(),vel)
        joints=dict(zip(self._arm.joint_names(),(joint_vels)))

        self._arm.set_joint_velocities(joints)
