#!/usr/bin/env python
import roslib
#roslib.load_manifest('ar_toolkit_detector')

import baxter
import baxter_interface
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ar_toolkit_detector.srv import *
from tf.transformations import *
import tf

from apriltags_ros.msg import AprilTagDetectionArray

from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import struct

print_targets=False

class EyeToHand(object):
    def __init__(self):
        self._marker_size=rospy.get_param('marker_size')
        self._num_targets=rospy.get_param('num_targets')
        self._move_on=rospy.get_param('move_on')
        self._test_mode=rospy.get_param('test_on')

        self._distance_offset=0

        self._ideal_xs=np.matlib.zeros((4*2,1))
        self._target_pos=np.matrix([[-1,-1,0],[-1,1,0],[1,1,0],[1,-1,0]])*self._marker_size/2
        self._ideal_pos_offset=np.transpose(np.matrix([[-1,0,-1,0,-1,0,-1,0]])*self._distance_offset)
        #self._target_pos=np.matrix([[0,0,0],[0.10266,0,0],[0.10266,0.10231,0],[0,0.10231,0]])

        self._target_set=False
        self._print_targets=False

        self._lambda=0.1
        self._dt=1/30
        self._max_vel=0.025
        self._max_ang=0.025
        self._marker_lost=False
        limb='left'

        self._arm=baxter_interface.limb.Limb(limb)
        cv2.namedWindow("Image window",0)
        self._bridge=CvBridge()
        self._image=None

        self._pub_rate=rospy.Publisher("robot/joint_state_publish_rate",UInt16)

        self._kin = baxter_kinematics('left')

        self._target_found=False
        self._start_servoing=False
        self._done=False

        #self._out=cv2.VideoWriter('~/Desktop/output.avi','XVID',10.0,(640,400))

        rospy.on_shutdown(self._shutdown_hook)
        
        baxter.open_head_cam()

        rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self._process_detections)
        rospy.Subscriber("/tag_detections_image",Image,self._get_image)

        print "Initialized!"
        print "Press any key to set the target position"

    def _project_2D(self,target):
        x=np.matlib.zeros((2,1))
        p=self._target_pos[target,:]-self._ideal_p
        x=np.matrix([p[0,0]/p[0,2],p[0,1]/p[0,2]])
        return x
    
    # Get the translation vector and rotation matrix from a pose message
    def _get_t_R(self,pose):
        t=np.transpose(np.matrix([pose.position.x,pose.position.y,pose.position.z]))
        quat=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R_full=quaternion_matrix(quat)
        R=R_full[0:3,0:3]
        return t,R
    
    # Gets corner points from a detection
    def _extract_corners(self,detection):
        target_xs=np.matlib.zeros((2*4,1))
        for i in range(0,4):
            target_xs[i*2,0]=detection.corners[i*2]
            target_xs[i*2+1,0]=detection.corners[i*2+1]

        return target_xs

    # Generate the transformation between frames for a twist (v,w)
    def _generate_frame_transform(self,t,R,isarm):
        r_t=-np.dot(np.transpose(R),t)
        skew_matrix=np.matrix([[0,-r_t[2],r_t[1]],[r_t[2],0,-r_t[0]],[-r_t[1],r_t[0],0]])
        if isarm:
            transform_top=np.concatenate((R,np.zeros((3,3))),axis=1)
        else:
            transform_top=np.concatenate((R,-np.dot(R,skew_matrix)),axis=1)
        transform_bottom=np.concatenate((np.zeros((3,3)),R),axis=1)
        transform=np.concatenate((transform_top,transform_bottom),axis=0)
        return transform
    
    # Generate the interaction matrix from the detection
    def _generate_L(self,detection):
        # Get marker position relative to camera
        (t,R)=self._get_t_R(detection.pose.pose)
        
        L=np.matlib.zeros((2*4,6))
        
        target_xs=self._extract_corners(detection)
        
        for i in range(0,4):
            x=target_xs[i*2,0]
            y=target_xs[i*2+1,0]
            
            position=np.dot(R,np.transpose(self._target_pos[i,:]))+t
            
            X=position[0,0]
            Y=position[1,0]
            Z=position[2,0]

            L[i*2:i*2+2,:]=np.matrix([[-1/Z,0,x/Z,x*y,-(1+x*x),y],[0,-1/Z,y/Z,1+y*y,-x*y,-x]])
            if (self._print_targets):
                print x,y

        # HIGHLY EXPERIMENTAL
        transform=self._generate_frame_transform(t,R,False)
        L=-np.dot(L,transform)
        return L
    
    def _retract_arm(self,rotation):
        vel=np.matrix([[0],[0],[-self._max_vel],[0],[0],[0]])
        vel[0:3]=np.dot(rotation,vel[0:3])
        self._set_vel(vel)

    def _set_vel(self,vel):
        joint_vels=np.dot(self._kin.jacobian_pseudo_inverse(),vel)

        joints=dict(zip(self._arm.joint_names(),(joint_vels)))
        self._arm.set_joint_velocities(joints)

    def _set_zero_vel(self):
        vel=np.matrix([[0],[0],[0],[0],[0],[0]])
        self._set_vel(vel)

    def _shutdown_hook(self):
        #self._set_zero_vel()
        #self._out.release()
        cv2.destroyAllWindows()
    
    # Check detections for target marker
    def _find_marker(self,detections,marker_num):
        for i in range(0,len(detections)):
            # If marker found
            if detections[i].id==marker_num:
                # Set target position
                return i
        return -1

    def _draw_targets(self,image):
        for i in range(0,4):
            cv2.circle(image,(int((self._ideal_xs[i*2,0]*205)+320),int((self._ideal_xs[i*2+1,0]*205)+200)),3,(0,0,255))

    # Allows the user to set the target position for the gripper
    def _target_initialization_selection(self,detections):
        print "runnign!"
        try:
            cv2.imshow("Image window",self._image)
            user_input=cv2.waitKey(50)
        except:
            pass
        if not detections:
            return
        print "Actual detection yay!"
        target_marker=self._find_marker(detections,1)
        print "Markers found"
        # Ignore if nothing pressed
        if user_input==-1:
            return
        # If marker not found, nothing more we can do here
        if target_marker==-1:
            print "Error: Target marker not found, please try again"
            return

        # Otherwise, set position
        self._ideal_xs=self._extract_corners(detections[target_marker])+self._ideal_pos_offset
        self._target_set=True
        print "Initialization complete! Press any key to begin visual servoing"
        return

    def _get_image(self,image):
        #print "Image received!"
        try:
            cv_image=self._bridge.imgmsg_to_cv2(image,"bgr8")
            self._image=cv_image
        except CvBridgeError, e:
            print e

    def _process_detections(self,apriltag_detections):
        #print "detection received"
        if not self._test_mode:
            # If we don't have the target position yet, see if we can set it
            if not self._target_set:
                self._target_initialization_selection(apriltag_detections.detections)
                return
            if not apriltag_detections.detections:
                #print "Nothing detected!"
                return

            # Wait for user to let us start the visual servoing
            if not self._start_servoing:
                user_input=cv2.waitKey(0)
                if user_input!=-1:
                    self._start_servoing=True
            
            # Otherwise, target set, user ok! Start servoing.
            target_marker=self._find_marker(apriltag_detections.detections,1)
            if target_marker!=-1:
                self._ideal_xs=self._extract_corners(apriltag_detections.detections[target_marker])+self._ideal_pos_offset
            
            my_marker=self._find_marker(apriltag_detections.detections,0)

            if my_marker==-1:
                print "Target marker not found"
                return
            
            self._draw_targets(self._image)
            cv2.imshow("Image window",self._image)
            #out.write(self._image)
            user_input=cv2.waitKey(10)
            if user_input!=-1:
                rospy.signal_shutdown("Done")

            (marker_pose,marker_R)=self._get_t_R(apriltag_detections.detections[my_marker].pose.pose)
        
            target_xs=self._extract_corners(apriltag_detections.detections[my_marker])
            L=self._generate_L(apriltag_detections.detections[my_marker])

            error=target_xs-self._ideal_xs
            print('Error is:')
            print np.linalg.norm(error)
        
            vel=-self._lambda*np.dot(np.linalg.pinv(L),error)
        
        if (self._test_mode):
            vel=np.matrix([[0],[0],[0],[0],[0],[0]])

        left_hand_pose=baxter.get_left_arm_pose()

        (arm_t,arm_R)=self._get_t_R(left_hand_pose)

        # TODO: cam_R here is transform from marker to gripper
        cam_R=np.zeros((3,3))
        cam_t=np.transpose(np.matrix([[0,0,-0.04]]))
        cam_R[0,2]=-1
        cam_R[1,0]=1
        cam_R[2,1]=-1
        cam_transform=self._generate_frame_transform(cam_t,(cam_R),False)
        #print cam_transform
        vel=np.dot(cam_transform,vel)
        #print vel
        #vel[3:6]=cam_R*vel[3:6]
        #vel[0:3]=cam_R*vel[0:3]+np.transpose(np.cross(cam_t,np.transpose(vel[3:6])))  
        
        transform=self._generate_frame_transform(arm_t,arm_R,True)
        vel=np.dot(transform,vel)

        if (np.linalg.norm(vel[0:3])>self._max_vel):
            vel=vel/np.linalg.norm(vel[0:3])*self._max_vel
            
        #vel=vel*2

        joint_vels=np.dot(self._kin.jacobian_pseudo_inverse(),vel)

        joints=dict(zip(self._arm.joint_names(),(joint_vels)))
        
        if (self._move_on):
            self._arm.set_joint_velocities(joints)
                
def main(args):
    rospy.init_node('marker_detector')
    eyeToHand=EyeToHand()
    if (eyeToHand._test_mode):
        rate=rospy.Rate(20)
        while not rospy.is_shutdown():
            eyeToHand._process_detections(None)
            rate.sleep()
    else:
        rospy.spin()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
