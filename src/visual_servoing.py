#!/usr/bin/env python
import roslib
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
from tf.transformations import *

from utility import *

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Point32,
    Quaternion,
)

import struct

class VisualServoing(object):
    def __init__(self, ibvs,marker_size=None):
        # Options
        if marker_size is not None:
            self._corner_pose = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]])*marker_size
        self._print_targets=False
        self._translation_only=False
        self._test_servoing=False
        self._L=np.matlib.zeros((2*4,6))
        self._ideal_feature=np.matlib.zeros((4*2,1))
        self._ibvs = ibvs

        self._lambda=0.5

        self._target_set=False
        
        print "Initialized!"

    def set_target(self,ideal_cam_pose, ideal_cam_rot, ideal_corners=None):
        self._ideal_cam_pose=ideal_cam_pose
        self._ideal_cam_rot = ideal_cam_rot
        if ideal_corners is not None:
            self._ideal_corners = ideal_corners
        if self._ibvs:
            self._eih_initialize_target_feature()
        if not self._ibvs:
            self._ideal_feature = self._calc_feature(ideal_cam_pose,ideal_cam_rot)
        self._target_set=True
 
    def _shutdown_hook(self):
        pass

    def _eih_initialize_target_feature(self):
        for i in range(0,4):
            x=self._ideal_corners[i*2]
            y=self._ideal_corners[i*2+1]
            self._ideal_feature[i*2,0]=x
            self._ideal_feature[i*2+1,0]=y
            p = self._ideal_cam_pose
            #p=np.dot(self._ideal_cam_rot[0:3,0:3].T,self._corner_pose[i,:])+self._ideal_cam_pose
            X=p[0]
            Y=p[1]
            Z=p[2]
            self._L[i*2:i*2+2,:]=np.matrix([[-1/Z,0,x/Z,x*y,-(1+x*x),y],[0,-1/Z,y/Z,1+y*y,-x*y,-x]])

    # Generate the interaction matrix from the detection
    def _generate_L(self,t,R):
        # Convert R to axis-angle representation
        (theta,u,p)=rotation_from_matrix(R)

        L_top=np.concatenate((-np.identity(3),generate_skew_mat(t)),axis=1)
        
        L_bottom=np.concatenate((np.zeros((3,3)),(np.identity(3)-theta/2*generate_skew_mat(u)+(1-(np.sinc(theta)/(np.sinc(theta/2)*np.sinc(theta/2))))*np.dot(generate_skew_mat(u),generate_skew_mat(u)))),axis=1)
        
        L=np.concatenate((L_top,L_bottom),axis=0)
        
        return L
    
    # Calculates the pbvs feature
    def _calc_feature(self,t,R):
        R_rotated=np.dot((self._ideal_cam_rot),R.T)
        (theta,u,p)=rotation_from_matrix(R_rotated)
        if self._translation_only:
            feature=np.concatenate((t[0:3,0],np.zeros((3,1))),axis=0)
        else:
            feature=np.concatenate((t[0:3,0],theta*u[:,None]),axis=0)
        return feature
    
    # Tag detection may have come in
    def get_next_vel(self,t=None,R=None,corners=None):
        if self._ibvs:
            target_feature = corners.flatten()
            target_feature = target_feature[:,None]
            L = self._L
        else:
            L = self._generate_L(t,R)
            target_feature = self._calc_feature(t,R)
        error = target_feature - self._ideal_feature

        vel=-self._lambda*np.dot(np.linalg.inv(L),error)

        return vel
