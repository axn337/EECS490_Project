#!/usr/bin/python

import rospy
import roslib
import sensor_msgs
#import opencv2
from cv_bridge import CvBridge
import std_msgs
import sys
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from reconstruction.msg import ImageStamped
from std_msgs.msg import Header
import numpy as np
from geometry_msgs.msg import Pose

def main():
	rospy.init_node('image_sender', anonymous=True)
	pub = rospy.Publisher("image_stamped", ImageStamped, queue_size = 1)
	br = CvBridge()
	pose1 = Pose()
	pose2 = Pose()
	img1 = cv2.imread('image_1.png')
	img2 = cv2.imread('image_2.png')

	img1_ros = br.cv2_to_imgmsg(img1)
	img2_ros = br.cv2_to_imgmsg(img2)

	pose1.position.x = 0.433012783527
	pose1.position.y = -0.249999880791
	pose1.position.z = 0.300000011921
	pose1.orientation.w = -0.226004580276
	pose1.orientation.x = 0.224733660268
	pose1.orientation.y = -0.0251856589319
	pose1.orientation.z = -0.947513796322

	pose2.position.x = -0.469846427441
	pose2.position.y = 0.171009778976
	pose2.position.z = 0.300000011921
	pose2.orientation.w = -0.955021155296
	pose2.orientation.x = -0.0593451678968
	pose2.orientation.y = -0.213517445104
	pose2.orientation.z = 0.197035643023

	msg1 = ImageStamped()
	msg2 = ImageStamped()

	msg1.image = img1_ros
	msg1.pose = pose1

	msg2.image = img2_ros
	msg2.pose = pose2

	while not rospy.is_shutdown():
		pub.publish(msg1)
		rospy.sleep(0.5)
		pub.publish(msg2)
		rospy.sleep(0.5)

if __name__ == '__main__':
	main()