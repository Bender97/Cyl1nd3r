import matplotlib.pyplot as plt
import time
import sys
import numpy as np
from nuscenes import NuScenes
from PIL import Image
from nuscenes.utils.data_classes import LidarSegPointCloud, LidarPointCloud
from nuscenes.eval.lidarseg.utils import ConfusionMatrix, LidarsegClassMapper

from nuscenes.lidarseg.lidarseg_utils import colormap_to_colors, plt_to_cv2, get_stats, \
	get_labels_in_coloring, create_lidarseg_legend, paint_points_label

from nuscenes.utils.geometry_utils import view_points

from nuscenes.utils.data_io import load_bin_file
import os
from pyquaternion import Quaternion
import tqdm



def map_pointcloud_to_image(pointsensor_token: str,
							camera_token: str,
							min_dist: float = 1.0,
							render_intensity: bool = False,
							show_lidarseg: bool = False,
							filter_lidarseg_labels = None,
							lidarseg_preds_bin_path: str = None,
							show_panoptic: bool = False) :
	"""
    Given a point sensor (lidar/radar) token and camera sample_data token, load pointcloud and map it to the image
    plane.
    :param pointsensor_token: Lidar/radar sample_data token.
    :param camera_token: Camera sample_data token.
    :param min_dist: Distance from the camera below which points are discarded.
    :param render_intensity: Whether to render lidar intensity instead of point depth.
    :param show_lidarseg: Whether to render lidar intensity instead of point depth.
    :param filter_lidarseg_labels: Only show lidar points which belong to the given list of classes. If None
        or the list is empty, all classes will be displayed.
    :param lidarseg_preds_bin_path: A path to the .bin file which contains the user's lidar segmentation
                                    predictions for the sample.
    :param show_panoptic: When set to True, the lidar data is colored with the panoptic labels. When set
        to False, the colors of the lidar data represent the distance from the center of the ego vehicle.
        If show_lidarseg is True, show_panoptic will be set to False.
    :return (pointcloud <np.float: 2, n)>, coloring <np.float: n>, image <Image>).
    """

	cam = nusc.get('sample_data', camera_token)
	pointsensor = nusc.get('sample_data', pointsensor_token)
	pcl_path = os.path.join(nusc.dataroot, pointsensor['filename'])
	if pointsensor['sensor_modality'] == 'lidar':
		if show_lidarseg or show_panoptic:
			gt_from = 'lidarseg' if show_lidarseg else 'panoptic'
			assert hasattr(nusc, gt_from), f'Error: nuScenes-{gt_from} not installed!'

			# Ensure that lidar pointcloud is from a keyframe.
			assert pointsensor['is_key_frame'], \
				'Error: Only pointclouds which are keyframes have lidar segmentation labels. Rendering aborted.'

			assert not render_intensity, 'Error: Invalid options selected. You can only select either ' \
										 'render_intensity or show_lidarseg, not both.'

		pc = LidarPointCloud.from_file(pcl_path)
		print("pointcloud size:", len(pc.points[0]))
	im = Image.open(os.path.join(nusc.dataroot, cam['filename']))

	print("0) points[0]:", pc.points[:, 0])

	# Points live in the point sensor frame. So they need to be transformed via global to the image plane.
	# First step: transform the pointcloud to the ego vehicle frame for the timestamp of the sweep.
	cs_record = nusc.get('calibrated_sensor', pointsensor['calibrated_sensor_token'])
	pc.rotate(Quaternion(cs_record['rotation']).rotation_matrix)
	pc.translate(np.array(cs_record['translation']))

	print("1) points[0]:", pc.points[:, 0])

	# Second step: transform from ego to the global frame.
	poserecord = nusc.get('ego_pose', pointsensor['ego_pose_token'])
	pc.rotate(Quaternion(poserecord['rotation']).rotation_matrix)
	pc.translate(np.array(poserecord['translation']))

	print("2) points[0]:", pc.points[:, 0])

	# Third step: transform from global into the ego vehicle frame for the timestamp of the image.
	poserecord = nusc.get('ego_pose', cam['ego_pose_token'])
	pc.translate(-np.array(poserecord['translation']))
	pc.rotate(Quaternion(poserecord['rotation']).rotation_matrix.T)

	print("rot matrix")
	print(Quaternion(poserecord['rotation']).rotation_matrix)
	print()
	print("T")
	print(Quaternion(poserecord['rotation']).rotation_matrix.T)
	print()

	print("3) points[0]:", pc.points[:, 0])

	# Fourth step: transform from ego into the camera.
	cs_record = nusc.get('calibrated_sensor', cam['calibrated_sensor_token'])
	pc.translate(-np.array(cs_record['translation']))
	pc.rotate(Quaternion(cs_record['rotation']).rotation_matrix.T)

	print("4) points[0]:", pc.points[:, 0])

	# Fifth step: actually take a "picture" of the point cloud.
	# Grab the depths (camera frame z axis points away from the camera).
	depths = pc.points[2, :]

	if render_intensity:
		assert pointsensor['sensor_modality'] == 'lidar', 'Error: Can only render intensity for lidar, ' \
														  'not %s!' % pointsensor['sensor_modality']
		# Retrieve the color from the intensities.
		# Performs arbitary scaling to achieve more visually pleasing results.
		intensities = pc.points[3, :]
		intensities = (intensities - np.min(intensities)) / (np.max(intensities) - np.min(intensities))
		intensities = intensities ** 0.1
		intensities = np.maximum(0, intensities - 0.5)
		coloring = intensities
	elif show_lidarseg or show_panoptic:
		assert pointsensor['sensor_modality'] == 'lidar', 'Error: Can only render lidarseg labels for lidar, ' \
														  'not %s!' % pointsensor['sensor_modality']

		gt_from = 'lidarseg' if show_lidarseg else 'panoptic'
		semantic_table = getattr(nusc, gt_from)

		if lidarseg_preds_bin_path:
			sample_token = nusc.get('sample_data', pointsensor_token)['sample_token']
			lidarseg_labels_filename = lidarseg_preds_bin_path
			assert os.path.exists(lidarseg_labels_filename), \
				'Error: Unable to find {} to load the predictions for sample token {} (lidar ' \
				'sample data token {}) from.'.format(lidarseg_labels_filename, sample_token, pointsensor_token)
		else:
			if len(semantic_table) > 0:  # Ensure {lidarseg/panoptic}.json is not empty (e.g. in case of v1.0-test).
				lidarseg_labels_filename = os.path.join(nusc.dataroot,
													nusc.get(gt_from, pointsensor_token)['filename'])
			else:
				lidarseg_labels_filename = None

		if lidarseg_labels_filename:
			# Paint each label in the pointcloud with a RGBA value.
			if show_lidarseg:
				coloring = paint_points_label(lidarseg_labels_filename,
											  filter_lidarseg_labels,
											  nusc.lidarseg_name2idx_mapping,
											  nusc.colormap)

		else:
			coloring = depths
			print(f'Warning: There are no lidarseg labels in {nusc.version}. Points will be colored according '
				  f'to distance from the ego vehicle instead.')
	else:
		# Retrieve the color from the depth.
		coloring = depths

	# Take the actual picture (matrix multiplication with camera-matrix + renormalization).
	# points = view_points(pc.points[:3, :], np.array(cs_record['camera_intrinsic']), normalize=True)

	points = pc.points[:3, :]
	view = np.array(cs_record['camera_intrinsic'])

	# print("view:", view)

	viewpad = np.eye(4)
	viewpad[:view.shape[0], :view.shape[1]] = view

	# print("viewpad:", viewpad)

	nbr_points = points.shape[1]

	# Do operation in homogenous coordinates.
	points = np.concatenate((points, np.ones((1, nbr_points))))
	points = np.dot(viewpad, points)
	points = points[:3, :]

	points = points / points[2:3, :].repeat(3, 0).reshape(3, nbr_points)

	print("5) points[0]:", points[:, 0])

	# Remove points that are either outside or behind the camera. Leave a margin of 1 pixel for aesthetic reasons.
	# Also make sure points are at least 1m in front of the camera to avoid seeing the lidar points on the camera
	# casing for non-keyframes which are slightly out of sync.
	mask = np.ones(depths.shape[0], dtype=bool)
	mask = np.logical_and(mask, depths > min_dist)
	mask = np.logical_and(mask, points[0, :] > 1)
	mask = np.logical_and(mask, points[0, :] < im.size[0] - 1)
	mask = np.logical_and(mask, points[1, :] > 1)
	mask = np.logical_and(mask, points[1, :] < im.size[1] - 1)
	points = points[:, mask]
	coloring = coloring[mask]

	print("6) mask  [0]:", mask[0])

	return points, coloring, im, depths[mask]


nusc = NuScenes(version='v1.0-mini', dataroot='/home/fusy/Documents/bin2pcd/v1.0-mini', verbose=True)


pointsensor_channel="LIDAR_TOP"
camera_channel="CAM_FRONT"
#f = open("playback_recipe.txt", "w")
ax = None
for my_sample in tqdm.tqdm(nusc.sample):
	pointsensor_token = my_sample['data'][pointsensor_channel]
	camera_token = my_sample['data'][camera_channel]

	points, coloring, im, depths = map_pointcloud_to_image(pointsensor_token, camera_token,
														render_intensity=False,
														show_lidarseg=True,
														filter_lidarseg_labels=None,
														lidarseg_preds_bin_path=None,
														show_panoptic=False)

	cam = nusc.get('sample_data', camera_token)
	pointsensor = nusc.get('sample_data', pointsensor_token)
	print("CAMERA_TOKEN:", camera_token)
	print("POINTS_TOKEN:", pointsensor_token)
	print()
	cs_record = nusc.get('calibrated_sensor', pointsensor['calibrated_sensor_token'])
	print(cs_record)
	print()
	poserecord = nusc.get('ego_pose', pointsensor['ego_pose_token'])
	print(poserecord)
	print()
	poserecord = nusc.get('ego_pose', cam['ego_pose_token'])
	print(poserecord)
	print()
	cs_record = nusc.get('calibrated_sensor', cam['calibrated_sensor_token'])
	print(cs_record)
	print()



	# Init axes.
	if ax is None:
		fig, ax = plt.subplots(1, 1, figsize=(9, 16))
		fig.canvas.set_window_title("prova")
	else:  # Set title on if rendering as part of render_sample.
		ax.set_title(camera_channel)
	ax.imshow(im)
	ax.scatter(points[0, :], points[1, :], c=coloring, s=5)
	ax.axis('off')
	plt.pause(0.5)

	plt.cla()
plt.show()



	
