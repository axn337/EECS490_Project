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


points = []
K = np.eye(3)

f1 = 1.0
pp1 = (0.0,0.0)
f2 = 1.0
pp2 = (0.0,0.0)
#obviously wrong, need ammar to tell me

K = np.eye(3)

#tunable params
MIN_MATCH_COUNT = 50

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


def pose_to_tf(pose):

	quat = pose.orientation

	tf_mat = np.zeros((4,4))

	tf_mat[0,0] = 1 - 2 * quat.y ** 2 - 2 ** quat.z ** 2 
	tf_mat[0,1] = 2 * quat.x * quat.y - 2 * quat.z * quat.w
	tf_mat[0,2] = 2 * quat.x * quat.z + 2 * quat.y * quat.w

	tf_mat[1,0] = 2 * quat.x * quat.y + 2 * quat.z * quat.w
	tf_mat[1,1] = 1 - 2 * quat.x ** 2 - 2 * quat.z ** 2
	tf_mat[1,2] = 2 * quat.y * quat.z - 2 * quat.x * quat.w

	tf_mat[2,0] = 2 * quat.x * quat.z - 2 * quat.y * quat.w 
	tf_mat[2,1] = 2 * quat.y * quat.z + 2 * quat.x * quat.w
	tf_mat[2,2] = 1 - 2 * quat.x ** 2 - 2 * quat.y ** 2

	tf_mat[0,3] = pose.position.x
	tf_mat[1,3] = pose.position.y
	tf_mat[2,3] = pose.position.z
	tf_mat[3,3] = 1
	return tf_mat

def make3d(img1, tfMat1, img2, tfMat2):
	
	projMat1 = np.dot(K, np.linalg.inv(tfMat1)[:3,:])
	projMat2 = np.dot(K, np.linalg.inv(tfMat2)[:3,:])

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
	flann = cv2.FlannBasedMatcher(index_params,search_params)
	matches = flann.knnMatch(des1,des2,k=K_neighbors)

	#storing good matches
	good = []

	for m,n in matches:
		if m.distance < dist_thresh * n.distance:
			good.append(m)

	if len(good) > MIN_MATCH_COUNT:
		src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
		dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

		src_pts_norm = cv2.undistortPoints(src_pts, cameraMatrix=K,distCoeffs=None)
		dst_pts_norm = cv2.undistortPoints(dst_pts, cameraMatrix=K,distCoeffs=None)

		E, mask = cv2.findEssentialMat(src_pts_norm,dst_pts_norm,focal=f1, pp=pp1,method=cv2.RANSAC,prob=0.999,threshold=3.0)
		points,R,t,mask = cv2.recoverPose(E,src_pts_norm,dst_pts_norm)

		#compare this tf to gazebo reported tf. any way to use these as estimates?
		M_src = np.hstack((R,t))
		M_dst = np.hstack((np.eye(3,3),np.zeros((3,1))))
		
		P_src = np.dot(K,M_src)
		P_dst = np.dot(K,M_dst)		

		points3d = cv2.triangulatePoints(P_src,P_dst,src_pts,dst_pts)
		#points3d = cv2.triangulatePoints(projMat1,projMat2,src_pts,dst_pts)
		points3d /= points3d[3]
		points_world = np.dot((np.linalg.inv(tfMat1)), points3d)[:3,:]
		return points_world
	else:
		return 0


def populatePointCloud(points):
	header = Header()
	#header.seq = g_seq
	#g_seq += 1
	
	header.frame_id = 'map'
	header.stamp = rospy.Time.now()
	cloud = pc2.create_cloud_xyz32(header,points)
	
	return cloud





def main():
	global points
	rospy.init_node('recon', anonymous=True)
	pub = rospy.Publisher("reconstructed_pts", PointCloud2, queue_size = 1)
	img1 = cv2.imread('image_1.png')
	img2 = cv2.imread('image_2.png')
	pose1 = Pose()
	pose2 = Pose()
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

	tfmat1 = pose_to_tf(pose1)
	tfmat2 = pose_to_tf(pose2)

	while not rospy.is_shutdown():
		pts = make3d(img1,tfmat1,img2,tfmat2)
		pts = np.asarray(pts,np.float32)
		for i in range(pts.shape[1]):
			points.append([pts[0,i],pts[1,i],pts[2,i]])
		cloud = populatePointCloud(points) 
		pub.publish(cloud)
		rospy.sleep(0.5)


if __name__ == '__main__':
	main()