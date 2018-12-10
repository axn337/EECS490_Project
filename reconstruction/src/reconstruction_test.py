#!/usr/bin/python

import rospy
import roslib
import sensor_msgs
#import opencv2
import cv_bridge
import std_msgs
import sys
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from reconstruction.msg import ImageStamped
import numpy as np

imgs = []
proj_mats = []

def CallBack(self, data):
	img = cv_bridge.imgmsg_to_cv2(data.image) #conversion from ROS msg format to cv2
	
	origin = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])


	quat = data.pose.orientation

	rot_mat = np.zeros(3,3)

	rot_mat[0,0] = 1 - 2 * quat.y ^ 2 - 2 * quat.z ^ 2 
	rot_mat[0,1] = 2 * quat.x * quat.y - 2 * quat.z * quat.w
	rot_mat[0,2] = 2 * quat.x * quat.z + 2 * quat.y * quat.w

	rot_mat[1,0] = 2 * quat.x * quat.y + 2 * quat.z * quat.w
	rot_mat[1,1] = 1 - 2 * quat.x ^ 2 - 2 * quat.z ^ 2
	rot_mat[1,2] = 2 * quat.y * quat.z - 2 * quat.x * quat.w

	rot_mat[2,0] = 2 * quat.x * quat.z - 2 * quat.y * quat.w 
	rot_mat[2,1] = 2 * quat.y * quat.z + 2 * quat.x * quat.w
	rot_mat[2,3] = 1 - 2 * quat.x ^ 2 - 2 * quat.y ^ 2

	proj_mat = np.block([[rot_mat, origin]])
	
	
	# for dbg purp
	print("recvd pose")
	print(rot_mat)
	print(origin)

	imgs.append(img)
	proj_mats.append(proj_mat)


def main():
	rospy.init_node('image_feature', anonymous=True)
	sub = rospy.Subscriber("image_stamped", ImageStamped, queue_size = 1)
	pub = rospy.Publisher("3d_pts", PointCloud2, queue_size = 1)
	#pts = PointCloud2()
	prev_cnt = 0
	while not rospy.is_shutdown():
		imgs_np = np.array(imgs)
		proj_mats_np = np.array(proj_mats)
		new_cnt = imgs_np.shape[0]
		if(imgs_np.shape[0] == proj_mats_np.shape[0]): # ensuring same number of imgs and mats
			#do img processing here 
			# TODO
			for i in range(prev_cnt, new_cnt):
				img1 = imgs_np[i]
				img2 = imgs_np[i+1]
				proj_mat1 = proj_mats_np[i]
				proj_mat2 = proj_mats_np[i+1]
				# do processing here 
				pub.publish(pts)
			prev_cnt += new_cnt


if __name__ == '__main__':
	main()



		 

