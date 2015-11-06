#!/usr/bin/env python
"""
Performs eye in hand (eih) image based visual servoing (ibvs). 
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
# Don't include these classes if you aren't using AprilTags or Baxter
from apriltag_client import AprilTagClient
from apriltags_ros.msg import AprilTagDetectionArray
from baxter_wrapper import BaxterVS


from visual_servoing import VisualServoing
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

class IbvsEih(object):
    """
    Performs eye in hand (eih) image based visual servoing (ibvs). 
    """
    def __init__(self):
        # Baxter specific code. You don't need this if you're using another robot.
        # Sets the arm that is used for the servoing
        limb='right'
        self._baxter = BaxterVS(limb)

        # AprilTag specific code. You don't need this if you're using another tracking system.
        # Initializes the marker that the arm should track
        target_marker = 0
        self._apriltag_client = AprilTagClient(target_marker)
        
        self._visual_servo = VisualServoing(ibvs=True)
    
    def new_image_arrived(self):
        """
        Boolean to test if a new image has arrived.
        """
        if self._apriltag_client.corners is not None:
            self._apriltag_client.corners = None
            return True
        return False

    def _get_detected_corners(self):
        """
        Returns the most recently detected corners in the image.
        """
        # This method currently uses AprilTag detections, so replace with
        # your own method if otherwise.

        return self._apriltag_client.corners
    
    def _command_velocity(self,vel):
        """
        Move the camera at the specified v and omega in vel (6x1 vector).
        """
        # This method currently commands the Baxter robot from Rethink Robotics.
        #Replace with your own method if using another manipulator.
        
        baxter_vel = self._baxter.cam_to_body(servo_vel)
        self._baxter.set_hand_vel(baxter_vel)
    
    def set_target(self,final_camera_depth,desired_corners):
        """
        Sets the final camera depth (Z) and desired position for the tracked features
        at the goal position.
        """
        ideal_cam_pose = np.array([0,0,Z])
        self._vision_servo.set_target(ideal_cam_pose,None,desired_corners)
    
    def move_to_position(self,final_camera_depth,desired_corners,dist_tol):
        """
        Runs one instance of the visual servoing control law. Call when a new
        image has arrived.
        """
        self.set_target(final_camera_depth,desired_corners)
        r = rospy.Rate(60)
        while np.linalg.norm(error)>dist_tol and not rospy.is_shutdown():
            if not self.new_image_arrived():
                continue
            
            # Continue if no corners detected
            marker_corners = self._get_detected_corners()
            if marker_corners is None:
                continue

            # Don't move if the target hasn't been set
            if not self._visual_servo._target_set:
                continue
            
            # Get control law velocity and transform to body frame, then send to robot
            servo_vel = self._visual_servo.get_next_vel(corners = marker_corners)
            self._command_velocity(servo_vel)
            r.sleep()

def main(args):
    rospy.init_node('ibvs_eih')
    ibvseih = IbvsEih()
    # Set desired camera depth and desired feature coordinates as well as distance from goal before stopping
    final_camera_depth = 0.1
    desired_corners = np.array([10,10,-10,10,10,-10,-10,-10])
    dist_tol = 0.5
    
    ibvseih.move_to_position(final_camera_depth,desired_corners,dist_tol)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
