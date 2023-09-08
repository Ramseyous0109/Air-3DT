import open3d as o3d
import numpy as np
import struct
from pathlib import Path
import os
from pypcd import pypcd
import io

def read_pcd_file(filename):
    pcd_data = pypcd.PointCloud.from_path(filename)
    points = np.zeros([pcd_data.width, 4], dtype=np.float32)
    points[:, 0] = pcd_data.pc_data['x'].copy()
    points[:, 1] = pcd_data.pc_data['y'].copy()
    points[:, 2] = pcd_data.pc_data['z'].copy()
    points[:, 3] = np.ones([pcd_data.width]).astype(np.float32)
    points = points * 10
    return points

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

def read_pfm(filename):
    with Path(filename).open('rb') as pfm_file:
        line1, line2, line3 = (pfm_file.readline().decode('latin-1').strip() for _ in range(3))
        print(line1)
        print(line2)
        print(line3)
        assert line1 in ('PF', 'Pf')
        channels = 3 if "PF" in line1 else 1
        width, height = (int(s) for s in line2.split())
        scale_endianess = float(line3)
        bigendian = scale_endianess > 0
        scale = abs(scale_endianess)
        buffer = pfm_file.read()
        samples = width * height * channels
        assert len(buffer) == samples * 4
        fmt = f'{"<>"[bigendian]}{samples}f'
        decoded = struct.unpack(fmt, buffer)
        shape = (height, width, 3) if channels == 3 else (height, width)
        return np.reshape(decoded, shape) * scale

def save_point_cloud_to_bin(point_cloud, filename):
    points = point_cloud.points
    colors = point_cloud.colors if point_cloud.has_colors() else None
    with open(filename, "wb") as f:
        for i in range(len(points)):
            point = points[i]
            f.write(struct.pack("fff", point[0], point[1], point[2]))
            if colors is not None:
                color = colors[i]
                f.write(struct.pack("BBB", int(color[0] * 255), int(color[1] * 255), int(color[2] * 255)))



def save_to_pcd(data_dir, pcd_dir, timestamp):
    K = np.array([
    [400,0,400],
    [0,400,300],
    [0,0,1]
    ])
    intrinsic = o3d.camera.PinholeCameraIntrinsic(800, 600, K[0,0], K[1,1], K[0,2], K[1,2])
    a = os.path.join(data_dir,timestamp+'.png')
    color = o3d.io.read_image(a)
    a = os.path.join(data_dir,timestamp+'.pfm')
    depth = read_pfm(a)
    scale = 100
    depth_array_scaled = (depth * scale).astype(np.uint16)
    depth = o3d.geometry.Image(depth_array_scaled)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    o3d.io.write_point_cloud(os.path.join(pcd_dir, timestamp+'.pcd'), pcd)
