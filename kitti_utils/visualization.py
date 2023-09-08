import numpy as np
import os
from scipy.spatial.transform import Rotation
import cv2

def draw_projected_box3d(image, qs, color=(0,255,0), thickness=4):
    ''' Draw 3d bounding box in image
        qs: (8,2) array of vertices for the 3d box in following order:
            1 -------- 0
           /|         /|
          2 -------- 3 .
          | |        | |
          . 5 -------- 4
          |/         |/
          6 -------- 7
    '''
    if qs is not None:
        qs = qs.astype(np.int32)
        for k in range(0,4):
           i,j=k,(k+1)%4
           image = cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness) # use LINE_AA for opencv3

           i,j=k+4,(k+1)%4 + 4
           image = cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness)

           i,j=k,k+4
           image = cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness)
    return image

#相机坐标系：右x, 下y，前z
#kitti坐标系：前x, 左y，上z
def transform_tf(points):
    camera_x = -points[:,1]
    camera_y = -points[:,2]
    camera_z = points[:,0]
    points[:,0] = camera_x
    points[:,1] = camera_y
    points[:,2] = camera_z
    return points

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

def get_box_vertices_bing(center, length, width, height, quat):
    # 将四元数转换为旋转矩阵
    rot = Rotation.from_quat(quat).as_matrix()
    
    # 计算box的半长、半宽、半高
    half_l = length / 2
    half_w = width / 2
    half_h = height / 2
    
    # 创建box的八个顶点的局部坐标
    local_vertices = np.array([
        [-half_l, -half_w, -half_h],
        [-half_l, +half_w, -half_h],
        [+half_l, +half_w, -half_h],
        [+half_l, -half_w, -half_h],
        [-half_l, -half_w, +half_h],
        [-half_l, +half_w, +half_h],
        [+half_l, +half_w, +half_h],
        [+half_l, -half_w, +half_h]
    ])
    
    # 将局部坐标旋转并平移到中心点
    global_vertices = local_vertices @ rot.T + center
    #print(local_vertices @ rot.T)
    # 返回全局坐标
    return global_vertices

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


camera_intrinsic = np.array([
    [400,0,400],
    [0,400,300],
    [0,0,1]
])

f_x = 400
image_width = 800
image_height = 600
ann_file = '/path/to/trk/label'
img_file = '/path/to/visualized/image'
f = open(ann_file)
img = cv2.imread(img_file)
img2 = np.copy(img)
for line in f:
    line = line.strip()
    array = line.split(' ')
    h,w,l = float(array[10]),float(array[11]),float(array[12])
    x,y,z = float(array[13]),float(array[14]),float(array[15])
    yaw = float(array[16])
    quat = euler_to_quaternion(0,0,yaw)
    box_vertices = get_box_vertices_bing([x,y,z], l, w, h, quat)
    vertice_2d = []
    for vertice in box_vertices:
        camera_point = project([-vertice[1], -vertice[2], vertice[0]], camera_intrinsic)
        vertice_2d.append(camera_point)
    box3d_pts_2d = np.array(vertice_2d)
    box_2d = calculate_2d_bbox(np.array(vertice_2d))
    X1, Y1 = int(box_2d[0][0]), int(box_2d[0][1])  # 第一个顶点坐标
    X2, Y2 = int(box_2d[1][0]), int(box_2d[1][1])  # 第二个顶点坐标
    center_2d = ((box_2d[1][0]+box_2d[0][0])/2, (box_2d[1][1]+box_2d[0][1])/2)
    offset = center_2d[0] - image_width / 2
    alpha = np.arctan2(offset, f_x)
    img2 = draw_projected_box3d(img2, box3d_pts_2d)
    cv2.rectangle(img, (X1, Y1), (X2, Y2), (0, 255, 0), 2)
cv2.imwrite('/path/anno2d.png', img)
cv2.imwrite('/path/anno3d.png', img2)