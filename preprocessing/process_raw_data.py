import numpy as np
import os
import pc2bin
data_dir = 'your/data/root'
image_path = os.path.join(data_dir, 'images')
index_file = os.path.join(data_dir, 'airsim_rec.txt')
os.makedirs(os.path.join(data_dir, 'pcd'), exist_ok=True)
os.makedirs(os.path.join(data_dir, 'processed_data'), exist_ok=True)
pcd_path = os.path.join(data_dir, 'pcd')
out_vehicle_file = os.path.join(data_dir, 'processed_data\\vehicle.txt')

stamps = []

f = open(index_file)
f_out = open(out_vehicle_file, 'a')
next(f)

for line in f:
    line = line.strip()
    array = line.split()
    stamps.append(int(array[1]))
f.close()
f = open(index_file)
next(f)
for line in f:
    line = line.strip()
    array = line.split()
    stamps.append(int(array[1]))
    position = [float(array[2]),float(array[3]),float(array[4])] #x,y,z
    orientation = [float(array[6]),float(array[7]),float(array[8]),float(array[5])] #x,y,z,w
    image_name = array[9]
    color_name = image_name.split(';')[0]
    depth_name = image_name.split(';')[1]
    write_data = f"{array[1]} {array[2]} {array[3]} {array[4]} {array[6]} {array[7]} {array[8]} {array[5]}\n"
    f_out.write(write_data)
    os.rename(os.path.join(image_path, depth_name), os.path.join(image_path, array[1]+'.pfm')) 
    os.rename(os.path.join(image_path, color_name), os.path.join(image_path, array[1]+'.png')) 
    #存pcd
f.close()
f_out.close()


f = open(index_file)
next(f)
for line in f:
    line = line.strip()
    stamps.append(int(line.split()[1]))
    pc2bin.save_to_pcd(image_path, pcd_path, line.split()[1])
f.close()





camera_pose_file = os.path.join(data_dir, 'camera_pose.txt')
f = open(camera_pose_file)
# 匹配时间戳
record_data_stamps = []
for line in f:
    line = line.strip()
    record_data_stamps.append(int(line.split()[0]))
stamps = np.array(stamps)
record_data_stamps = np.array(record_data_stamps)
idx = (np.abs(record_data_stamps - stamps[:, None])).argmin(axis=1)
filtered_data_stamps = record_data_stamps[idx]
stamp_pairs = {}
for i in range(len(stamps)):
    stamp_pairs[filtered_data_stamps[i]] = stamps[i]
f.close()


f = open(camera_pose_file)
out_camera_file = os.path.join(data_dir, 'processed_data\\camera.txt')
f_out = open(out_camera_file, 'a')
for line in f:
    line = line.strip()
    array = line.split()
    stamp = int(array[0])
    if stamp not in stamp_pairs:
        continue
    changed_stamp = stamp_pairs[stamp]
    data = f"{changed_stamp} {array[1]} {array[2]} {array[3]} {array[4]} {array[5]} {array[6]} {array[7]}\n"
    f_out.write(data)
f_out.close()

f = open(os.path.join(data_dir, 'imu.txt'))
out_imu_file = os.path.join(data_dir, 'processed_data\\imu.txt')
f_out = open(out_imu_file, 'a')
for line in f:
    line = line.strip()
    array = line.split()
    stamp = int(array[0])
    if stamp not in stamp_pairs:
        continue
    changed_stamp = stamp_pairs[stamp]
    data = f"{changed_stamp} {array[1]} {array[2]} {array[3]} {array[4]} {array[5]} {array[6]}\n"
    f_out.write(data)
f_out.close()

object_file = os.path.join(data_dir, 'object.txt')
out_object_file = os.path.join(data_dir, 'processed_data\\object.txt')
f = open(object_file)
f_out = open(out_object_file, 'a')
for line in f:
    line = line.strip()
    array = line.split()
    stamp = int(array[0])
    if stamp not in stamp_pairs:
        continue
    changed_stamp = stamp_pairs[stamp]
    data = f"{changed_stamp} {array[1]} {array[2]} {array[3]} {array[4]} {array[5]} {array[6]} {array[7]} {array[8]} {array[9]} {array[10]} {array[11]}\n"
    f_out.write(data)
f.close()
f_out.close()
print('ddd')
