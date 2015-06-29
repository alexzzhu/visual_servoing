#!/usr/bin/env python
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
    def __init__(self):
        self._baxter = BaxterVS('right')
        target_marker = 0

        self._apriltag_client = AprilTagClient(target_marker)
        self._visual_servo = VisualServoing(False)    
        cv2.namedWindow("Detected tags",0)
                    
    def _main_loop(self):
        key = -1
        image = self._apriltag_client._image
        if image is None:
            return
        if self._visual_servo._target_set:
            corners = self._visual_servo._ideal_corners
            for i in range(0,4):
                cv2.circle(image,(int(corners[i*2]*407.19+317.22),int(corners[i*2+1]*407.195786+206.752)),5,(255,0,0),5)
        cv2.imshow("Detected tags",image)
        key = cv2.waitKey(5)
        
        marker_pose = self._apriltag_client.marker_t
        marker_rot = self._apriltag_client.marker_R
        marker_corners = self._apriltag_client.corners
        if marker_pose is None:
            return
        
        if self._visual_servo._target_set:
            self._apriltag_client.marker_t = None
            self._apriltag_client.marker_R = None
            self._apriltag_client.corners = None
        
        if key !=-1:
            rospy.loginfo("Setting new target")
            desired_corners = marker_corners
            desired_pose = marker_pose
            desired_rot = marker_rot
            self._visual_servo.set_target(desired_pose,desired_rot,desired_corners)

        if not self._visual_servo._target_set:
            return
        servo_vel = self._visual_servo.get_next_vel(marker_pose,marker_rot)
        baxter_vel = self._baxter.cam_to_body(servo_vel)
        self._baxter.set_hand_vel(baxter_vel)

def main(args):
    rospy.init_node('marker_detector')
    pbvseih = PbvsEih()
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        pbvseih._main_loop()
        r.sleep()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
