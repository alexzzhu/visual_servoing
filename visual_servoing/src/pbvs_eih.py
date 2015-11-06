#!/usr/bin/env python
"""
Performs eye in hand (eih) pose based visual servoing (pbvs). 
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""
import roslib
import numpy as np
import numpy.matlib
import sys
import rospy
import cv2
from tf.transformations import *
import tf
from apriltag_client import AprilTagClient
from baxter_wrapper import BaxterVS
from visual_servoing import VisualServoing
from apriltags_ros.msg import AprilTagDetectionArray

from utility import *

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

class PbvsEih(object):
    """
    Performs eye in hand (eih) image based visual servoing (ibvs). 
    """
    def __init__(self):
        # Sets the arm that is used for the servoing
        limb='right'
        self._baxter = BaxterVS(limb)

        # Initializes the marker that the arm should track
        target_marker = 0
        self._apriltag_client = AprilTagClient(target_marker)

        self._visual_servo = VisualServoing(False)    
        cv2.namedWindow("Detected tags",0)
                    
    def _main_iter(self):
        """
        Runs one instance of the visual servoing control law. Call in a loop.
        """
        key = -1
        image = self._apriltag_client.image
        if image is None:
            return
        # Draw the ideal position if selected
        if self._visual_servo._target_set:
            corners = self._visual_servo._ideal_corners
            for i in range(0,4):
                # Camera intrinsics currently hard coded, tune as needed, 
                # or add a subscriber to the camera_info message.
                cv2.circle(image,(int(corners[i*2]*407.19+317.22),int(corners[i*2+1]*407.195786+206.752)),5,(255,0,0),5)
        cv2.imshow("Detected tags",image)
        key = cv2.waitKey(5)
        
        # Return if no tag detected
        marker_pose = self._apriltag_client.marker_t
        marker_rot = self._apriltag_client.marker_R
        marker_corners = self._apriltag_client.corners
        if marker_pose is None:
            return

        # Don't reprocess detections
        if self._visual_servo._target_set:
            self._apriltag_client.marker_t = None
            self._apriltag_client.marker_R = None
            self._apriltag_client.corners = None
        
            # Press any key to set a new target position for the servo control law.
        if key !=-1:
            rospy.loginfo("Setting new target")
            desired_corners = marker_corners
            desired_pose = marker_pose
            desired_rot = marker_rot
            self._visual_servo.set_target(desired_pose,desired_rot,desired_corners)

        if not self._visual_servo._target_set:
            return

        # Get control law velocity and transform to body frame, then send to baxter
        servo_vel = self._visual_servo.get_next_vel(marker_pose,marker_rot)
        baxter_vel = self._baxter.cam_to_body(servo_vel)
        self._baxter.set_hand_vel(baxter_vel)

def main(args):
    rospy.init_node('pbvs_eih')
    pbvseih = PbvsEih()
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        pbvseih._main_iter()
        r.sleep()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
