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
np.set_printoptions(suppress=True)
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
    line_sets = []
    for object in object_pose_dir[timestamp]:
        a = object_pose_dir[timestamp]
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
    return box_sets, line_sets, size_sets, yaw_sets, box_2d, alpha

def convert_to_kitti_annotation(box_vertices, size, yaw, box_2d, alpha):
    # 计算中心坐标
    center = np.mean(box_vertices, axis=0)
    Xc, Yc, Zc = center
    # 计算长、宽、高
    width, length, height = size
    #line = f"{Xc} {Yc} {Zc} {width} {length} {height} {yaw} Pedestrian\n"
    Zc = Zc - height / 2 # Bottom center defined in KITTI dataset
    line = f"Pedestrian 0.00 0 {alpha} {box_2d[0][0]} {box_2d[0][1]} {box_2d[1][0]} {box_2d[1][1]} {height} {width} {length} {Xc} {Yc} {Zc} {yaw}\n"
    return line

detection_dataset_id = 0
detection_timestamps = []
out_dir = '/your/output/dir'

imgset_train_f = open(os.path.join(out_dir, 'ImageSets/train.txt'), 'a')
imgset_test_f = open(os.path.join(out_dir, 'ImageSets/test.txt'), 'a')
imgset_val_f = open(os.path.join(out_dir, 'ImageSets/val.txt'), 'a')
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
    for pcd_file in os.listdir(os.path.join(root_dir, data_dir, pcd_dir)):
        a = random.randint(0,9)
        if a == 1:
            out_data_dir = os.path.join(out_dir, 'testing')
        elif a == 0:
            out_data_dir = os.path.join(out_dir, 'validation')
        else:
            out_data_dir = os.path.join(out_dir, 'training')
        out_bin_dir = os.path.join(out_data_dir, 'velodyne')
        out_img_dir = os.path.join(out_data_dir, 'image_2')
        out_label_dir = os.path.join(out_data_dir, 'label_2')
        out_calib_dir = os.path.join(out_data_dir, 'calib')
        timestamp = int(pcd_file.split('.')[0])
        points = read_pcd_file(os.path.join(root_dir, data_dir, pcd_dir, pcd_file))
        points = transform_tf(points)
        points_box, line_sets, size_sets, yaw_sets, box_2d, alpha = transform_label(points, obj_pose_stamp, camera_pose_dir, timestamp)
        #print(len(points_box))
        if len(points_box) != 0:
            name = '{:06d}'.format(detection_dataset_id)
            if a == 1:
                imgset_test_f.write(name+'\n')
            elif a == 0:
                imgset_val_f.write(name+'\n')
            else:
                imgset_train_f.write(name+'\n')
            img_file = os.path.join(root_dir, data_dir,'images',str(timestamp)+'.png')
            img = cv2.imread(img_file)
            cv2.imwrite(os.path.join(out_img_dir, name+'.png'), img)
            calib_file = open(os.path.join(out_calib_dir, name+'.txt'),'a')
            for line in calib_lines:
                calib_file.write(line)
            calib_file.close()
            with open(os.path.join(out_bin_dir, name+'.bin'), 'wb') as f_bin:
                f_bin.write(points.tobytes())
                f_bin.close()
            label_file = open(os.path.join(out_label_dir, name+'.txt'),'a')
            for box in points_box:
                box_vertices = points_box[box]
                box_size = size_sets[box]
                yaw = yaw_sets[box]
                line = convert_to_kitti_annotation(box_vertices, box_size, yaw, box_2d, alpha)
                print(line)
                label_file.write(line)
            label_file.close()
            detection_dataset_id = detection_dataset_id + 1
        
imgset_test_f.close()
imgset_train_f.close()
imgset_val_f.close()


