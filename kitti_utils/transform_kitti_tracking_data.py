import os
import cv2
import numpy as np
from pypcd import pypcd
from utils import *

img_out_dir = '/your/out/dir/image_02'
calib_out_dir = '/your/out/dir/calib'
root_dir = '/your/root/dir'
#data_dirs = ['scene0/seq0','scene0/seq1','scene0/seq2','scene0/seq3','scene1/seq0','scene1/seq1','scene1/seq2','scene2/seq0','scene2/seq1']
data_dirs = ['scene3/seq0','scene3/seq1','scene3/seq2','scene3/seq3','scene3/seq4','scene3/seq5','scene3/seq6']
velodyne_out_dir = '/your/out/dir/velodyne'
scene_index = 9
for data_dir in data_dirs:   
    os.makedirs(os.path.join(img_out_dir, '{:04d}'.format(scene_index)), exist_ok=True)
    os.makedirs(os.path.join(calib_out_dir, '{:04d}'.format(scene_index)), exist_ok=True)
    frame_index = 0
    for img_file in sorted(os.listdir(os.path.join(root_dir, data_dir, 'images/'))):
        if img_file.split('.')[1] == 'pfm':
            continue
        img = cv2.imread(os.path.join(root_dir, data_dir, 'images/',img_file))
        name = '{:06d}'.format(frame_index)
        cv2.imwrite(os.path.join(img_out_dir, '{:04d}'.format(scene_index), name+'.png'), img)
        frame_index = frame_index + 1
    os.makedirs(os.path.join(velodyne_out_dir, '{:04d}'.format(scene_index)), exist_ok=True)
    frame_index = 0
    for velo_file in sorted(os.listdir(os.path.join(root_dir, data_dir, 'pcd/'))):
        points = read_pcd_file(os.path.join(root_dir, data_dir, 'pcd/',velo_file))
        points = transform_tf(points)
        name = '{:06d}'.format(frame_index)
        with open(os.path.join(velodyne_out_dir, '{:04d}'.format(scene_index), name+'.bin'), 'wb') as f_bin:
            f_bin.write(points.tobytes())
            f_bin.close()
        frame_index = frame_index + 1
    scene_index = scene_index + 1