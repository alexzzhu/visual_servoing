#!/usr/bin/env python
import roslib
roslib.load_manifest('ar_toolkit_detector')

import baxter
import baxter_interface
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
from image_subscriber import image_subscriber
from ar_toolkit_detector.srv import *
from tf.transformations import *
import tf

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

class VisualServo(object):
    def __init__(self):
        self._num_targets=rospy.get_param('/num_targets')
        self._move_on=rospy.get_param('move_on')
        self._grasp_on=rospy.get_param('grasp_on')
        self._ideal_xs=np.matlib.zeros((4*2,1))
        self._L=np.matlib.zeros((2*4,6))
        self._ic=image_subscriber()
        rospy.wait_for_service('/left/detect_patt')
        self._detector=rospy.ServiceProxy('/left/detect_patt',detect_patt)
        self._target_pos=np.matrix([[0,0,0],[0.1784,0,0],[0,0.1149,0],[0.1784,0.1149,0]])
        #self._target_pos=np.matrix([[0,0,0],[0.10266,0,0],[0.10266,0.10231,0],[0,0.10231,0]])

        self._ideal_p=np.matrix('0.038,0.05,-0.07')
        #self._ideal_p=np.matrix('0.038,0.05,-0.1')   
        self._lambda=0.1
        self._dt=1/30
        self._max_vel=0.025
        self._max_ang=0.025
        self._marker_lost=False
        limb='left'

        self._ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(self._ns, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()
        self._arm=baxter_interface.limb.Limb(limb)
        self._pub_rate=rospy.Publisher("robot/joint_state_publish_rate",UInt16)

        self._kin = baxter_kinematics('left')

        self._target_found=False
        self._done=False

        rospy.on_shutdown(self._shutdown_hook)
        
        baxter.open_left_arm_cam()

    def _project_2D(self,target):
        x=np.matlib.zeros((2,1))
        p=self._target_pos[target,:]-self._ideal_p
        x=np.matrix([p[0,0]/p[0,2],p[0,1]/p[0,2]])
        return x

    def _initialize_ideal_pos(self):
        xs=np.matrix([[-0.1080617, -0.034723],[0.5763489,-0.034340266],[0.57975244,0.650942087173],[-0.10924192,0.6495161]])
        for i in range(0,4):#self._num_targets):
            #xs=self._project_2D(i)
            print xs
            #xs=np.matrix([[
            x=xs[i,0]
            y=xs[i,1]
            self._ideal_xs[i*2,0]=x
            self._ideal_xs[i*2+1,0]=y
            p=self._target_pos[i,:]-self._ideal_p
            X=p[0,0]
            Y=p[0,1]
            Z=p[0,2]
            self._L[i*2:i*2+2,:]=np.matrix([[-1/Z,0,x/Z,x*y,-(1+x*x),y],[0,-1/Z,y/Z,1+y*y,-x*y,-x]])
            #print('Ideal coords are:\n')
            #print self._ideal_xs
            #print self._L
    
    def _get_ik(self,pose):
        self._ikreq.pose_stamp.append(pose)
        try:
            rospy.wait_for_service(self._ns, 5.0)
            resp = self._iksvc(self._ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                self._ikreq.SEED_USER: 'User Provided Seed',
                self._ikreq.SEED_CURRENT: 'Current Joint Angles',
                self._ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                }.get(resp_seeds[0], 'None')
        #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
        #(seed_str,))
        # Format solution into Limb API-compatible dictionary
            return dict(zip(resp.joints[0].name, resp.joints[0].position))
        
        #arm.move_to_joint_positions(limb_joints)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            return None

    def _retract_arm(self,rotation):
        vel=np.matrix([[0],[0],[-self._max_vel],[0],[0],[0]])
        vel[0:3]=rotation*vel[0:3]
        self._set_vel(vel)

    def _set_vel(self,vel):
        joint_vels=self._kin.jacobian_pseudo_inverse()*vel

        joints=dict(zip(self._arm.joint_names(),(joint_vels)))
        self._arm.set_joint_velocities(joints)

    def _set_zero_vel(self):
        vel=np.matrix([[0],[0],[0],[0],[0],[0]])
        self._set_vel(vel)

    def _shutdown_hook(self):
        self._set_zero_vel()

    def apply_servoing(self):
        r=rospy.Rate(60)
        while not self._ic.updated:
            pass
        while (not baxter.open_left_arm_gripper()):
            pass
        self._initialize_ideal_pos()
        transform=baxter.get_tf_listener()
        transform.waitForTransform('/left_hand','left_hand_camera',rospy.Time(),rospy.Duration(4.0))
        (cam_t,cam_R)=transform.lookupTransform('/left_hand','/left_hand_camera',rospy.Time(0))

        cam_R_full=quaternion_matrix(cam_R)
        cam_R=cam_R_full[0:3,0:3]
        #temp=raw_input('Press enter to start IBVS\n')
        while not rospy.is_shutdown():
            self._pub_rate.publish(60)
            if (not self._target_found):
                try:
                    detections=self._detector(self._num_targets,1,self._ic.image)
                except:
                    print "Detection failed"
                    continue

                my_pose=baxter.get_arm_pose(baxter.LEFT)
                t=np.matrix([my_pose.position.x,my_pose.position.y,my_pose.position.z])
                quat=[my_pose.orientation.x, my_pose.orientation.y, my_pose.orientation.z, my_pose.orientation.w]
                R_full=quaternion_matrix(quat)
                R=R_full[0:3,0:3]

                target_xs=np.matlib.zeros((2*4,1))

                lost_marker=False

                for i in range(0,4):
                    center_x=detections.coordinates.coordinates[0].x
                    if (center_x==2048):
                        lost_marker=True
                        break;
                    x=detections.coordinates.coordinates[0].vertices[2*i]
                    y=detections.coordinates.coordinates[0].vertices[2*i+1]
                    
                    target_xs[i*2,0]=x
                    target_xs[i*2+1,0]=y
                    if (print_targets):
                        #print(target_xs[i*2,0],self._ideal_xs[i*2,0])
                        #print(target_xs[i*2+1,0],self._ideal_xs[i*2+1,0])
                        print x,y
                #if lost_marker:
                #    if (not self._marker_lost):
                #        self._marker_lost=True
                #        print('Marker lost\n')
                #        self._set_zero_vel()
                #    r.sleep()
                #    continue
                self._marker_lost=False

                transform=np.reshape(detections.transform,(3,4))

                rot=transform[0:3,0:3]
                transl=transform[0:3,3]

                #print rot                                                                                                                                                        
                #print transl                                                                                                                                                     
                print transform
                print ""

                error=target_xs-self._ideal_xs
             #print('Error is:')
                #print np.linalg.norm(error)
                
                if (self._grasp_on and np.linalg.norm(error)<0.1 and self._move_on):
                    self._target_found=True
            
                vel=-self._lambda*np.linalg.pinv(self._L)*error
                #vel=np.matrix([[0],[0],[0],[0],[0],[0]])

                #vel[3:6]=R*vel[3:6]
                #vel[0:3]=R*vel[0:3]#+np.transpose(np.cross(np.transpose(vel[3:6]),t))

                vel[3:6]=cam_R*vel[3:6]
                vel[0:3]=cam_R*vel[0:3]+np.transpose(np.cross(cam_t,np.transpose(vel[3:6])))  

                vel[3:6]=R*vel[3:6]
                vel[0:3]=R*vel[0:3]#+np.transpose(np.cross(t,np.transpose(vel[3:6])))

                if (np.linalg.norm(vel[0:3])>self._max_vel):
                    vel=vel/np.linalg.norm(vel[0:3])*self._max_vel
                vel=vel*5

                #vel=np.matrix([[0.1],[0],[0],[0],[0],[0]])

                joint_vels=self._kin.jacobian_pseudo_inverse()*vel

                joints=dict(zip(self._arm.joint_names(),(joint_vels)))
                if (self._move_on):
                    print "moving!"
                    self._arm.set_joint_velocities(joints)
            elif(not self._done):
                print "Gripping!"
                if (self._move_on):
                    baxter.close_left_arm_gripper()
                self._done=True
                rospy.signal_shutdown("Done")

            r.sleep()
            

def main(args):
    rospy.init_node('marker_detector')
    visualServo=VisualServo()
    visualServo.apply_servoing()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
