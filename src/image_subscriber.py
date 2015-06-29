#!/usr/bin/env python
import roslib
roslib.load_manifest('visual_servo')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_subscriber:
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic",Image,self.convert_image)
        self.updated=False
	self.cv_image=None

    def convert_image(self,data):
        self.image=data
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.updated=True
        except CvBridgeError, e:
            print e

        (rows,cols,channels) = self.cv_image.shape

        #cv2.imshow("Image window", self.cv_image)
        #cv2.waitKey(3)
