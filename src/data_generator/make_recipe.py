import matplotlib.pyplot as plt
import time
import sys
import numpy as np
from nuscenes import NuScenes

from nuscenes.utils.data_classes import LidarSegPointCloud
from nuscenes.eval.lidarseg.utils import ConfusionMatrix, LidarsegClassMapper

from nuscenes.lidarseg.lidarseg_utils import colormap_to_colors, plt_to_cv2, get_stats, \
	get_labels_in_coloring, create_lidarseg_legend, paint_points_label

from nuscenes.utils.geometry_utils import view_points

from nuscenes.utils.data_io import load_bin_file

from pyquaternion import Quaternion
import tqdm



def quat2str(quat):
	return str(quat[0]) + " " + str(quat[1]) + " " + str(quat[2]) + " " + str(quat[3])
def tran2str(tran):
	return str(tran[0]) + " " + str(tran[1]) + " " + str(tran[2])

def rot2str(rot):
	return str(rot[0][0]) + " " + str(rot[0][1]) + " " + str(rot[0][2]) + " " + str(rot[1][0]) + " " + str(rot[1][1]) + " " + str(rot[1][2]) + " " + str(rot[2][0]) + " " + str(rot[2][1]) + " " + str(rot[2][2])

def record2str(record):
	return rot2str(Quaternion(record['rotation']).rotation_matrix) + " " + tran2str(record['translation'])

def cameraintrinsic2str(record):
	return tran2str(record['camera_intrinsic'][0]) + " " + tran2str(record['camera_intrinsic'][1]) + " " + tran2str(record['camera_intrinsic'][2])

def mat4x42str(mat):
	return str(mat[0][0]) + " " + str(mat[0][1]) + " " + str(mat[0][2]) + " " + str(mat[0][3]) + " " + \
		   str(mat[1][0]) + " " + str(mat[1][1]) + " " + str(mat[1][2]) + " " + str(mat[1][3]) + " " + \
		   str(mat[2][0]) + " " + str(mat[2][1]) + " " + str(mat[2][2]) + " " + str(mat[2][3]) + " " + \
		   str(mat[3][0]) + " " + str(mat[3][1]) + " " + str(mat[3][2]) + " " + str(mat[3][3])

def mat3x32str(mat):
	return str(mat[0][0]) + " " + str(mat[0][1]) + " " + str(mat[0][2]) + " " + \
		   str(mat[1][0]) + " " + str(mat[1][1]) + " " + str(mat[1][2]) + " " + \
		   str(mat[2][0]) + " " + str(mat[2][1]) + " " + str(mat[2][2])


nusc = NuScenes(version='v1.0-mini', dataroot='/home/fusy/Documents/bin2pcd/v1.0-mini', verbose=True)

print(nusc.sample[0]['data']['LIDAR_TOP'])


f = open("playback_recipe.txt", "w")

for my_sample in tqdm.tqdm(nusc.sample):
	sample_data_token 				= my_sample['data']['LIDAR_TOP']
	camera_token_front 				= my_sample['data']['CAM_FRONT']
	camera_token_front_left 		= my_sample['data']['CAM_FRONT_LEFT']
	camera_token_front_right 		= my_sample['data']['CAM_FRONT_RIGHT']
	camera_token_back 				= my_sample['data']['CAM_BACK']
	camera_token_back_left 			= my_sample['data']['CAM_BACK_LEFT']
	camera_token_back_right 		= my_sample['data']['CAM_BACK_RIGHT']



	cam_front = nusc.get('sample_data', camera_token_front)
	cam_front_left = nusc.get('sample_data', camera_token_front_left)
	cam_front_right = nusc.get('sample_data', camera_token_front_right)
	cam_back = nusc.get('sample_data', camera_token_back)
	cam_back_left = nusc.get('sample_data', camera_token_back_left)
	cam_back_right = nusc.get('sample_data', camera_token_back_right)


	# Retrieve sensor & pose records
	sd_record = nusc.get('sample_data', sample_data_token)
	cs_record = nusc.get('calibrated_sensor', sd_record['calibrated_sensor_token'])


	root_path 	= "/home/fusy/Documents/bin2pcd/v1.0-mini/"
	pcl_path    = root_path + nusc.get('sample_data', sample_data_token)['filename']
	labels_path = root_path + nusc.get('lidarseg', sample_data_token)['filename']

	ep_record = nusc.get('ego_pose', sd_record['ego_pose_token'])


	# MATRICES for image projection
	# 1
	#cs_record_1 = nusc.get('calibrated_sensor', sd_record['calibrated_sensor_token'])
	# 2
	#poserecord_2 = nusc.get('ego_pose', sd_record['ego_pose_token'])
	# 3
	poserecord_3 = nusc.get('ego_pose', cam_front['ego_pose_token'])
	# 4
	cs_record_4 = nusc.get('calibrated_sensor', cam_front['calibrated_sensor_token'])



	cs_transmat = Quaternion(cs_record['rotation']).transformation_matrix
	cs_transmat[:3, 3] = np.array(cs_record['translation'])

	ep_transmat = Quaternion(ep_record['rotation']).transformation_matrix
	ep_transmat[:3, 3] = np.array(ep_record['translation'])

	mult = np.matmul(np.linalg.inv(cs_transmat), np.matmul(ep_transmat, cs_transmat))


	cam_trans = Quaternion(cs_record['rotation']).transformation_matrix
	cam_trans[:3, 3] = np.array(cs_record['translation'])


	f.write(pcl_path + "\n")
	f.write(labels_path + "\n")
	for cam in [cam_front, cam_front_left, cam_front_right, cam_back, cam_back_left, cam_back_right]:
		f.write(root_path + cam['filename'] + "\n")

	f.write(mat4x42str(mult) + "\n")


	pr_transmat = Quaternion(poserecord_3['rotation']).transformation_matrix
	pr_transmat[:3, 3] = np.array(poserecord_3['translation'])

	c4_transmat = Quaternion(cs_record_4['rotation']).transformation_matrix
	c4_transmat[:3, 3] = np.array(cs_record_4['translation'])



	f.write(  mat3x32str(cs_record_4['camera_intrinsic']) + "\n")
	f.write(  mat4x42str(cs_transmat) 	+ "\n")
	f.write(  mat4x42str(ep_transmat) 	+ "\n")
	f.write(  mat4x42str(pr_transmat) 	+ "\n")
	f.write(  mat4x42str(c4_transmat) 	+ "\n")

f.close()
