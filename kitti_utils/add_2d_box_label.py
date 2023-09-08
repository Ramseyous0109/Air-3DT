import numpy as np
import os
from scipy.spatial.transform import Rotation
import cv2
from utils import *

#calculate the 2d project bbox from 3D label

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    
    参数：
    roll: 绕X轴的旋转角度（以弧度为单位）
    pitch: 绕Y轴的旋转角度（以弧度为单位）
    yaw: 绕Z轴的旋转角度（以弧度为单位）
    
    返回值：
    四元数表示 [qw, qx, qy, qz]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]

image_height = 600
results_dir = '/your/directory/results'
output_dir = '/your/directory/label_02'
files = os.listdir(results_dir)
for file in files:
    ann_file = os.path.join(results_dir, file)
    output_file = os.path.join(output_dir, file)
    f = open(ann_file)
    o_f = open(output_file, 'a')
    for line in f:
        line = line.strip()
        array = line.split(',')
        h,w,l = float(array[7]),float(array[8]),float(array[9])
        x,y,z = float(array[10]),float(array[11]),float(array[12])
        yaw = float(array[13])
        quat = euler_to_quaternion(0,0,yaw)
        box_vertices = get_box_vertices_bing([x,y,z], l, w, h, quat)
        vertice_2d = []
        for vertice in box_vertices:
            camera_point = project([-vertice[1], -vertice[2], vertice[0]], camera_intrinsic)
            vertice_2d.append(camera_point)
        box_2d = calculate_2d_bbox(np.array(vertice_2d))
        X1, Y1 = int(box_2d[0][0]), int(box_2d[0][1]) 
        X2, Y2 = int(box_2d[1][0]), int(box_2d[1][1])
        center_2d = ((box_2d[1][0]+box_2d[0][0])/2, (box_2d[1][1]+box_2d[0][1])/2)
        offset = center_2d[0] - image_width / 2
        alpha = np.arctan2(offset, f_x)
        line = f'{array[0]},1,{X1},{Y1},{X2},{Y2},{array[6]},{array[7]},{array[8]},{array[9]},{array[10]},{array[11]},{array[12]},{array[13]},{alpha}\n'
        o_f.write(line)
    f.close()
    o_f.close()

