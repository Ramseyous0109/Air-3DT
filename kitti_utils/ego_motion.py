import numpy as np
import os
from scipy.spatial.transform import Rotation

# generate ego-motion file for KITTI tracking dataset

def euler_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll_rad), -np.sin(roll_rad)],
                    [0, np.sin(roll_rad), np.cos(roll_rad)]])
    
    R_y = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                    [0, 1, 0],
                    [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    
    R_z = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                    [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                    [0, 0, 1]])
    rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))
    return rotation_matrix

root_dir = '/root/dir'
data_dirs = ['scene0/seq0','scene0/seq1','scene0/seq2','scene0/seq3','scene1/seq0','scene1/seq1','scene1/seq2','scene2/seq0','scene2/seq1','scene3/seq0','scene3/seq1','scene3/seq2','scene3/seq3','scene3/seq4','scene3/seq5','scene3/seq6']
pose_file = 'processed_data/camera.txt'
out_dir = '/your/data/for/tracking/ego_motion'
seq = 0
for data_dir in data_dirs:
    f = open(os.path.join(root_dir, data_dir, pose_file))
    last_t = None
    last_rotation_matrix = None
    ego_motion = []
    for line in f:
        #timestamp x y z x y z w
        line = line.strip()
        line = line.split()
        x,y,z = float(line[1]), float(line[2]), -float(line[3])
        [yaw, pitch, roll] = Rotation.from_quat([float(line[4]),float(line[5]),float(line[6]),float(line[7])]).as_euler('zyx', degrees=False)
        yaw = -yaw
        rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)
        if last_t is None:
            matrix = np.identity(4)
        else:
            t = np.array([x,y,z]) - last_t
            r = np.dot(np.transpose(last_rotation_matrix), rotation_matrix)
            matrix = np.hstack((r, np.reshape(t, (3,1))))
            matrix = np.vstack((matrix, [[0,0,0,1]]))
        last_t = np.array([x,y,z])
        last_rotation_matrix = rotation_matrix    
        ego_motion.append(matrix)
    np.save(os.path.join(out_dir, '{:04d}'.format(seq)+'.npy'), np.array(ego_motion))
    seq = seq + 1
