import airsim
import time
import os
import keyboard

#This script acquire kinematics including camera and object pose, this script should run in parallel with recording sensor data in AirSim

client = airsim.MultirotorClient()
client.confirmConnection()
data_path = 'D:\Work\AirSim\AirSimDataset\process_data\\v1_0\scene3\seq6' #change to your specialized data path
#record camera data
stamp = client.getMultirotorState().timestamp
camera_file = open(os.path.join(data_path, 'camera_pose.txt'),'a')
object_file = open(os.path.join(data_path, 'object.txt'),'a')
object_list = ['a_manuel','a_eric','a_claudia','a_nathan','a_sophia','a_carla'] # change to the name of objects you want to track in unreal engine
sizes = { # change to the size of objects you want to track in unreal engine
    'a_carla': (66,169,59),
    'a_claudia': (72,180,55),
    'a_eric': (78, 183, 61),
    'a_manuel': (78, 186, 64),
    'a_nathan': (74, 184, 64),
    'a_sophia': (64, 176, 56)
}
while True:
    if keyboard.is_pressed('k'):
        break
    timestamp = client.getMultirotorState().timestamp
    pose = client.simGetCameraInfo(0).pose
    #acquire object pose
    obj_pose_list = []
    for object in object_list:
        obj_pose = client.simGetObjectPose(object)
        obj_pose_list.append((object, obj_pose))
    timestamp = int(timestamp / 1000000)
    camera_data = f"{timestamp} {pose.position.x_val} {pose.position.y_val} {pose.position.z_val} {pose.orientation.x_val} {pose.orientation.y_val} {pose.orientation.z_val} {pose.orientation.w_val}\n"
    for obj_pose in obj_pose_list:
        w, h, l=sizes[obj_pose[0]]
        pose_data = f"{timestamp} {obj_pose[0]} {obj_pose[1].position.x_val} {obj_pose[1].position.y_val} {obj_pose[1].position.z_val} {obj_pose[1].orientation.x_val} {obj_pose[1].orientation.y_val} {obj_pose[1].orientation.z_val} {obj_pose[1].orientation.w_val} {w} {h} {l}\n"
        object_file.write(pose_data)
    camera_file.write(camera_data)
camera_file.close()
object_file.close()
