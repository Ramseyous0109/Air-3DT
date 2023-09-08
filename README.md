## Air-3DT
Source code and data for the paper: 

Air-3DT: A Dataset for UAV-based Indoor 3D Object Detection and Tracking



#### Data Download:

- Google Drive: Coming Soon...
- Baidu Netdisk: Coming Soon...



#### Data Structure:

- Raw Data From AirSim (take sequence 0 of scene 0 as an example):

```bash
data/v1_0/scene0/seq0
|-- airsim_rec.txt #recording results, including vehicle pose and its correspoinding sensor data files
|-- camera_pose.txt #recorded by preprocessing/record_data.py
|-- images
|   |-- img_UAV0_0_0_1685445001271517300.pfm #depth file
|   |-- img_UAV0_0_0_168544500127117300.png #rgb image file
|   |-- ......
|-- object.txt #recorded by preprocessing/record_data.py
|-- imu.txt #recorded by preprocessing/record_data.py; optional 
```



- After Running  `preprocessing/record_data.py`

```bash
data/v1_0/scene0/seq0
|-- airsim_rec.txt
|-- bin #point cloud data formated .bin
|   |-- 1685445001271.bin
|   |-- 1685445001491.bin
|-- camera_pose.txt
|-- images
|   |-- 1685445001271.pfm #img raw data after synchronization
|   |-- 1685445001271.png
|-- object.txt
|-- pcd #point cloud data formated .bin
|   |-- 1685445001271.bin
|   |-- 1685445001491.bin
|-- imu.txt #recorded by preprocessing/record_data.py; optional 
|-- processed_data
    |-- camera.txt #camera pose data after synchronization
    |-- object.txt	#object pose data after synchronization
    |-- vehicle.txt #vehicle pose data after synchronization
    |-- imu.txt #imu data after synchronization; optional
```



#### KITTI Format Transformation:

- Transform to KITTI 3D Object Detection Format

```bash
python kitti_utils/transform_kitti_detection.py
```

​	Therefore, the KITTI-formatted detection dataset is generated

- Transform to KITTI 3D Object Tracking Format

  You should split Training, Validation and Test sets by yourself

```bash
# transform data to KITTI format
python kitti_utils/transform_kitti_tracking_data.py
# transform label to KITTI format
python kitti_utils/transform_kitti_tracking_label.py
# add 2d annotation to labels
python kitti_utils/add_2d_box_label.py
# generate ego_motion data
python kitti_utils/ego_motion.py
```

Therefore we generate dataset in KITTI 3D Object Tracking Format

```
data/v1_0/tracking/
|-- calib
|   |-- 0000.txt
|   |-- 0001.txt
|   |-- 0002.txt
|   |-- ......
|-- ego_motion
|   |-- 0000.npy
|   |-- 0001.npy
|   |-- 0002.npy
|   |-- ......
|-- image_02
|   |-- 0000
|   |  |-- 000000.png
|   |  |-- 000001.png
|   |  |-- 000002.png
|   |-- 0001
|   |-- 0002
|   |-- ......
|-- label_02
|   |-- 0000.txt
|   |-- 0001.txt
|   |-- 0002.txt
|   |-- ......
|-- velodyne
   |-- 0000
   |  |-- 000000.bin
   |  |-- 000001.bin
   |  |-- 000002.bin
   |-- 0001
   |-- 0002
   |-- ......
```

