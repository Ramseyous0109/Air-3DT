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
from utils import *

def transform_label(points, ar, c, timestamp):
    #cv2.imshow("test_img", a_img)
    #cv2.waitKey(0)
    #print(img_list)
    camera_pos = camera_pose_dir[timestamp]
    #x,y,z
    camera_pose_t = np.array([float(camera_pos[1]), float(camera_pos[2]), float(camera_pos[3])])
    #x,y,z,w
    camera_pose_ori = np.array([float(camera_pos[4]), float(camera_pos[5]), float(camera_pos[6]),float(camera_pos[7])])
    lines_box = np.array(edges)
    size_sets = {}
    box_sets = {}
    yaw_sets = {}
    boxes_2d = {}
    alphas = {}
    line_sets = []
    for object in object_pose_dir[timestamp]:
        obj_name = object[1]
        obj_pose_t = np.array([float(object[2]), float(object[3]), float(object[4])])
        obj_pose_ori = np.array([float(object[5]), float(object[6]), float(object[7]),  float(object[8])])
        w = float(object[9]) / 100
        h = float(object[10]) / 100 + 0.3
        l = float(object[11]) / 100
        #get 8 vertices
        vertices = get_box_vertices_bing(obj_pose_t, l, w, h, obj_pose_ori)
        yaw_camera = Rotation.from_quat(camera_pose_ori).as_euler('zyx', degrees=False)[0]
        yaw_obj = Rotation.from_quat(obj_pose_ori).as_euler('zyx', degrees=False)[0]
        yaw = yaw_obj - yaw_camera
        camera_vertices_3d = []
        camera_points = []
        for vertice in vertices:
            vertice_transformed = world_to_camera(vertice, camera_pose_t, camera_pose_ori)
            vertice_camera = [vertice_transformed[1], vertice_transformed[2], vertice_transformed[0]]
            camera_point = project(vertice_camera, camera_intrinsic)
            camera_points.append(camera_point)    
            camera_vertices_3d.append(vertice_camera)
            #print(camera_point)
        box_2d = calculate_2d_bbox(np.array(camera_points))
        center_2d = ((box_2d[1][0]+box_2d[0][0])/2, (box_2d[1][1]+box_2d[0][1])/2)
        offset = center_2d[0] - image_width / 2
        alpha = np.arctan2(offset, f_x)
        points_box = np.divide(np.array(camera_vertices_3d), 1)
        points_box = transform_tf(points_box)
        if legal_label(points, points_box):
            colors_box = np.array([color[obj_name][:3] for i in range(len(lines_box))])
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points_box)
            line_set.lines = o3d.utility.Vector2iVector(lines_box)
            line_set.colors = o3d.utility.Vector3dVector(colors_box)
            line_sets.append(line_set)
            box_sets[obj_name] = points_box
            size_sets[obj_name] = (w,l,h)
            yaw_sets[obj_name] = yaw
            boxes_2d[obj_name] = box_2d
            alphas[obj_name] = alpha
    return box_sets, line_sets, size_sets, yaw_sets, boxes_2d, alphas



def convert_to_kitti_annotation(frame_id, box_name, box_vertices, size, yaw, box_2d, alpha):
    # 计算中心坐标
    person_id = {
    'a_manuel': 0,
    'a_eric': 1,
    'a_claudia': 2,
    'a_nathan': 3,
    'a_carla': 5,
    'a_sophia': 4
    }
    box_id = person_id[box_name]
    center = np.mean(box_vertices, axis=0)
    Xc, Yc, Zc = center
    # 计算长、宽、高
    width, length, height = size
    #line = f"{Xc} {Yc} {Zc} {width} {length} {height} {yaw} Pedestrian\n"
    #Zc = Zc - height / 2 # Bottom center defined in KITTI dataset
    
    line_tracking = f"{frame_id} {box_id} Pedestrian 0.00 0 {alpha} {box_2d[0][0]} {box_2d[0][1]} {box_2d[1][0]} {box_2d[1][1]} {height} {width} {length} {Xc} {Yc} {Zc} {yaw}\n"
    line_detection = f"{frame_idx},1,0,0,0,0,1,{height},{width},{length},{Xc},{Yc},{Zc},{yaw},0\n"
    return line_tracking, line_detection


out_dir = '/home/OpenPCDet/data/v1_0/tracking/tracking_labels'
out_det_dir = '/home/OpenPCDet/data/v1_0/tracking/detection_labels'
i = 0
for data_dir in data_dirs:
    # read camera pose data
    f = open(os.path.join(root_dir, data_dir, 'processed_data/camera.txt'))
    camera_pose_dir = {}
    for line in f:
        line = line.strip()
        array = line.split()
        timestamp = int(array[0])
        camera_pose_dir[timestamp] = array
    f.close()

    #read object pose data
    f = open(os.path.join(root_dir, data_dir, 'processed_data/object.txt'))
    object_pose_dir = {}
    last_timestamp = 0
    obj_pose_stamp = []
    for line in f:
        line = line.strip()
        array = line.split()
        timestamp = int(array[0])
        if timestamp != last_timestamp:
            object_pose_dir[last_timestamp] = obj_pose_stamp
            obj_pose_stamp = []
            last_timestamp = timestamp
            obj_pose_stamp.append(array)
        else:
            obj_pose_stamp.append(array)
    object_pose_dir[timestamp] = obj_pose_stamp
    f.close()
    seq_name = '{:04d}'.format(i)
    #label_file = open(os.path.join(out_dir, seq_name+'.txt'),'a')
    detection_label_file = open(os.path.join(out_det_dir, seq_name+'.txt'),'a')
    frame_idx = 0
    pcd_list = sorted(os.listdir(os.path.join(root_dir, data_dir, pcd_dir)))
    for pcd_file in pcd_list:
        timestamp = int(pcd_file.split('.')[0])
        points = read_pcd_file(os.path.join(root_dir, data_dir, pcd_dir, pcd_file))
        points = transform_tf(points)
        points_box, line_sets, size_sets, yaw_sets, boxes_2d, alphas = transform_label(points, obj_pose_stamp, camera_pose_dir, timestamp)
        #print(len(points_box))
        if len(points_box) != 0:
            for box in points_box:
                box_vertices = points_box[box]
                box_size = size_sets[box]
                yaw = yaw_sets[box]
                box_2d = boxes_2d[box]
                alpha = alphas[box]
                line_tracking, line_detection = convert_to_kitti_annotation(frame_idx, box, box_vertices, box_size, yaw, box_2d, alpha)
                #label_file.write(line)
                detection_label_file.write(line_detection)
        frame_idx = frame_idx + 1
    i = i + 1
    #label_file.close()