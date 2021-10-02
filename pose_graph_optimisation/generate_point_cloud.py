# Use point only if uncertainty is below threshold

import numpy as np
import open3d as o3d
from params import index_matrix_2,camera_matrix_inv

def generate_point_cloud(keyframes,world_poses):
	points = []
	for i in keyframes:
		depth_reshaped = np.dstack((i.D, np.zeros_like(i.D), np.zeros_like(i.D)))
		depth_reshaped = np.reshape(depth_reshaped,(480*640,3))
		point_in_cam_frame = np.transpose(depth_reshaped*index_matrix_2) # 3x480*640
		points_in_world = np.matmul(camera_matrix_inv,point_in_cam_frame) # 3x480*640
		points_in_world = np.transpose(points_in_world) # 480*640x3
		# points_colours = np.reshape(i.I,(480*640,3)) # RGB values for each point
		# points.append(points_in_world,points_colours)
		points.append(points_in_world)
	return points


def convert_to_o3d_format(points):

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)

	return pcd

