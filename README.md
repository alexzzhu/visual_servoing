# visual-servoing
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)

This package provides classes to perform both image based and pose based visual servoing on the Rethink Robotics Baxter platform. At present, the apriltag fiducial system is used as a marker for tracking image features and poses, although another detection system can be switched out easily in the ibvs_eih.py and pbvs_eih.py classes. 

To run this package, you must have the included apriltags_ros package (or a similar apriltags package for ROS), the [PyKDL package for Baxter](http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL) and a set of python functions to control the Baxter. The current package that is used is developed at the GRASP Lab at the University of Pennsylvania, and is not included in this package. If you do not have access to this package, you simply need to replicate two functions:

* baxter.open_right_arm_cam_small() - Opens the right hand camera on the Baxter at half resolution and publishes the images and camera_info to ROS. The resolution can be tuned as you'd like, although at full resolution the apriltags_ros package approaches 1Hz. If you are using your own image topics, please update the apriltags_ros entry in the launch files.

* baxter.get_arm_camera_pose('right') - Gets the transformation between the right hand camera and the robot base, so that a velocity in the camera frame can be applied (as Baxter velocity commands are in base frame). Returns a homogeneous translation vector and rotation matrix. 

Both functions are used in baxter_wrapper.py.

As of June 29, 2015, both eye in hand (eih) image based visual servoing (ibvs) and eih pose based visual servoing (pbvs) are supported, although there is a bug in the pbvs so that rotation of the arm does not converge properly, and so the arm is only able to match the desired pose, not rotation.

For more information about the control theory used in this package, please see:

Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches." Robotics & Automation Magazine, IEEE 13.4 (2006): 82-90.
