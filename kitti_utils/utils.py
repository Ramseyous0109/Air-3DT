import numpy as np
import math
import os
import cv2
from scipy.spatial.transform import Rotation
import open3d as o3d
import struct
from pathlib import Path
from pypcd import pypcd
import random

root_dir = '/home/dataset/airsim/v1_0'
data_dirs = ['scene0/seq0','scene0/seq1','scene0/seq2','scene0/seq3','scene1/seq0','scene1/seq1','scene1/seq2','scene2/seq0','scene2/seq1']
pcd_dir = 'pcd/'
annotation_file = 'processed_data/object.txt'
camera_intrinsic = np.array([
    [400,0,400],
    [0,400,300],
    [0,0,1]
])
f_x = 400
image_width = 800
image_height = 600
color = {
    'a_manuel': [1,0,0,1],
    'a_eric': [0,1,0,1],
    'a_claudia': [0,0,1,1],
    'a_nathan': [1,1,0,1],
    'a_carla': [0,0,1,1],
    'a_sophia': [1,1,0,1]
}
edges = [(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]

calib_lines = [
    'P0: 400 0 400 0 0 400 300 0 0 0 1 0\n',
    'P1: 400 0 400 0 0 400 300 0 0 0 1 0\n',
    'P2: 400 0 400 0 0 400 300 0 0 0 1 0\n',
    'P3: 400 0 400 0 0 400 300 0 0 0 1 0\n',
    'R_rect 1 0 0 0 1 0 0 0 1\n',
    'Tr_velo_cam 1 0 0 0 0 1 0 0 0 0 1 0\n',
    'Tr_imu_velo 1 0 0 0 0 1 0 0 0 0 1 0\n'
]

def calculate_2d_bbox(projected_points):
    # Find the minimum and maximum x, y coordinates
    min_x = np.min(projected_points[:, 0])
    max_x = np.max(projected_points[:, 0])
    min_y = np.min(projected_points[:, 1])
    max_y = np.max(projected_points[:, 1])

    # Compute the 2D bounding box coordinates
    bbox_2d = [(min_x, min_y), (max_x, max_y)]

    return bbox_2d


def project(p_c, K):
    p_i = K @ p_c
    x = p_i[0] / p_i[2]
    y = p_i[1] / p_i[2]
    return np.array([x,y])

def world_to_camera(point_world, camera_position, camera_rotation):
    rotation = Rotation.from_quat(camera_rotation)
    point_camera = rotation.inv().apply(point_world - camera_position)
    return point_camera

def get_box_vertices_bing(center, length, width, height, quat):
    rot = Rotation.from_quat(quat).as_matrix()
    half_l = length / 2
    half_w = width / 2
    half_h = height / 2
    local_vertices = np.array([
        [-half_l, -half_w, -half_h],
        [-half_l, -half_w, +half_h],
        [-half_l, +half_w, -half_h],
        [-half_l, +half_w, +half_h],
        [+half_l, -half_w, -half_h],
        [+half_l, -half_w, +half_h],
        [+half_l, +half_w, -half_h],
        [+half_l, +half_w, +half_h]
    ])
    global_vertices = local_vertices @ rot.T + center
    return global_vertices

def read_pcd_file(filename):
    pcd_data = pypcd.PointCloud.from_path(filename)
    points = np.zeros([pcd_data.width, 4], dtype=np.float32)
    points[:, 0] = pcd_data.pc_data['x'].copy()
    points[:, 1] = pcd_data.pc_data['y'].copy()
    points[:, 2] = pcd_data.pc_data['z'].copy()
    points[:, 3] = np.ones([pcd_data.width]).astype(np.float32)
    #with open(os.path.join(data_dir, 'test.bin'), 'wb') as f:
        #f.write(points.tobytes())
    points = points * 10
    return points

#outlier removal
def legal_label(point_cloud, box_vertices):
    is_inside = np.logical_and.reduce((
            point_cloud[:, 0] >= np.min(box_vertices[:, 0]),
            point_cloud[:, 0] <= np.max(box_vertices[:, 0]),
            point_cloud[:, 1] >= np.min(box_vertices[:, 1]),
            point_cloud[:, 1] <= np.max(box_vertices[:, 1]),
            point_cloud[:, 2] >= np.min(box_vertices[:, 2]),
            point_cloud[:, 2] <= np.max(box_vertices[:, 2])
        ))
    num_points_inside = np.sum(is_inside)
    if num_points_inside >= 100:
        return True
    return False

#original camera：right x, down y，front z
#kitti coordinate：frontx, lefty，upz
def transform_tf(points):
    kitti_x = points[:,2]
    kitti_y = -points[:,0]
    kitti_z = -points[:,1]
    points[:,0] = kitti_x
    points[:,1] = kitti_y
    points[:,2] = kitti_z
    return points

