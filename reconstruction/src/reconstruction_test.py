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
from sensor_msgs.msg import PointCloud2, PointField
from reconstruction.msg import ImageStamped
from std_msgs.msg import Header
import numpy as np


imgs = []
tf_mats = []
pts = []

g_seq = 0

#camera params
f1 = 1.0
pp1 = (0.0,0.0)
f2 = 1.0
pp2 = (0.0,0.0)
#obviously wrong, need ammar to tell me


#tunable params
MIN_MATCH_COUNT = 5

#sift params
sift_N_octave_layers = 10
sift_edge_threshold = 100
sift_sigma = 1

#flann params
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees=10)
search_params = dict(checks=50)
K_neighbors = 2
dist_thresh = 0.7

FIELDS = [
	PointField(name='x', offset=0,datatype=7,count=1),
	PointField(name='y', offset=4,datatype=7,count=1),
	PointField(name='z', offset=8,datatype=7,count=1),
	]



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
	tf_mats.append(proj_mat)

def make3d(img1, tfMat1, img2, tfMat2):
	#projection matrices are P = K * [R|t] where K is a 3x3 of the intrinsic params of each camera
	res_img_sift1 = img1
	res_img_sift2 = img2
	#implementign sift feature extraction
	sift = cv2.xfeatures2d.SIFT_create(nOctaveLayers=sift_N_octave_layers, edgeThreshold=sift_edge_threshold, sigma=sift_sigma) #implemeting sift feature extraction
	kp1,des1 = sift.detectAndCompute(img1,None)
	kp2,des2 = sift.detectAndCompute(img2,None)
	#to visualize while dbg
	# img_sift1 = cv2.drawKeyPoints(img1,kp1,None,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	#implementing FLANN matching of sift features
	flann = cv2.FlannBasedMatches(index_params,search_params)
	matches = flann.knnMatch(des1,des2,k=K_neighbors)

	#storing good matches
	good = []

	for m,n in matches:
		if m.distance < dist_thresh * n.distance:
			good.append(m)

	if len(good) > MIN_MATCH_COUNT:
		src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(2,-1)
		dst_pts = np.float32([kp2[m.queryIdx].pt for m in good]).reshape(2,-1)

		src_pts_norm = cv2.undistortPoints(src_pts, cameraMatrix=K1,distCoeffs=None)
		dst_pts_norm = cv2.undistortPoints(dst_pts, cameraMatrix=K2,distCoeffs=None)

		E, mask = cv2.findEssentialMat(src_pts_norm,dst_pts_norm,focal=1.0, pp=(0.,0.),method=cv2.RANSAC,prob=0.999,threshold=3.0)
		points,R,t,mask = cv2.recoverPose(E,src_pts_norm,dst_pts_norm)

		#compare this tf to gazebo reported tf. any way to use these as estimates?
		M_src = np.hstack((R,t))
		M_dst = np.hstack((np.eye(3,3),np.zeros((3,1))))
		
		P_src = np.dot(K_src,M_src)
		P_dst = np.dot(K_dst,M_dst)		

		points3d = cv2.triangulatePoints(P_src,P_dst,src_pts,dst_pts)

		points3d /= points3d[3]
		return points3d
	else:
		return 0

def populatePointCloud(points3d):
	header = Header()
	#header.seq = g_seq
	#g_seq += 1
	header.frame_id = 'map'
	header.stamp = rospy.Time.now()
	cloud = pc2.create_cloud_xyz32(header,points3d)
	return cloud


def main():
	rospy.init_node('image_feature', anonymous=True)
	sub = rospy.Subscriber("image_stamped", ImageStamped, queue_size = 1)
	pub = rospy.Publisher("reconstructed_pts", PointCloud2, queue_size = 1)

	#for dbg 
	pts.append([1.0,2.0,3.0])

	prev_cnt = 0
	while not rospy.is_shutdown():
		imgs_np = np.array(imgs)
		tf_mats_np = np.array(tf_mats)
		new_cnt = imgs_np.shape[0]
		
		# TODO
		for i in range(prev_cnt, new_cnt):
			img1 = imgs_np[i]
			img2 = imgs_np[i+1]
			proj_mat1 = tf_mats_np[i]
			proj_mat2 = tf_mats_np[i+1]
			
			recon_pts = make3d(img1,proj_mat1,img2,proj_mat2)
			if not recon_pts == 0:
				pts.append(recon_pts[:,0:2])
		
		cloud = populatePointCloud(pts) 
		pub.publish(cloud)
		prev_cnt += new_cnt


if __name__ == '__main__':
	main()



		 

