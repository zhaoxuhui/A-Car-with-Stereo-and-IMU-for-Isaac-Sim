# coding=utf-8
from ctypes import c_void_p
from re import I
import rosbag
import subprocess, yaml
import sys
import os
import numpy as np
import cv2
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import subprocess, yaml


def loadTopicImgs(bag_path, topic_name):
    imgs = []
    timestamps = []

    counter = 0
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == topic_name:
                try:
                    cur_time = msg.header.stamp.to_sec()
                    print(cur_time)
                    timestamps.append(msg.header.stamp)

                    imgs.append(msg)
                    counter += 1
                    print(topic_name,counter)
                except CvBridgeError as e:
                    print(e)
    return imgs, timestamps

def loadOdometry(bag_path, topic_name):
    poses = []
    timestamps = []

    counter = 0
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == topic_name:
                cur_time = msg.header.stamp.to_sec()
                print(cur_time)
                timestamps.append(msg.header.stamp)

                poses.append(msg)
                counter += 1
                print(topic_name,counter)
    return poses, timestamps

def loadIMU(imu_path):
    imu_msg = Imu()
    angular_v = Vector3()
    linear_a = Vector3()

    imu_msgs = []
    timestamps = []

    counter = 0
    fin = open(imu_path, 'r')
    fin.readline()
    line = fin.readline().strip()
    while line:
        parts = line.split("\t")
        ts = float(parts[0])
        ax = float(parts[1])
        ay = float(parts[2])
        az = float(parts[3])
        wx = float(parts[4])
        wy = float(parts[5])
        wz = float(parts[6])

        imu_ts_ros = rospy.rostime.Time.from_sec(ts)
        timestamps.append(imu_ts_ros)
        imu_msg.header.stamp = imu_ts_ros
        
        angular_v.x = wx
        angular_v.y = wy
        angular_v.z = wz

        linear_a.x = ax
        linear_a.y = ay
        linear_a.z = az

        imu_msg.angular_velocity = angular_v
        imu_msg.linear_acceleration = linear_a

        imu_msgs.append(imu_msg)

        print("imu",counter)
        counter += 1

        line = fin.readline().strip()
    fin.close()
    return imu_msgs, timestamps


def getSummaryInfo(bag_path):
    info_strs = []
    info_dict = yaml.load(
        subprocess.Popen(['rosbag', 'info', '--yaml', bag_path], stdout=subprocess.PIPE).communicate()[0])
    end_timestamp = float(info_dict['end'])
    duration = float(info_dict['duration'])
    start_timestamp = end_timestamp - duration

    start_time_str = "Start timestamp:" + str(start_timestamp) + " s"
    end_time_str = "End timestamp:" + str(end_timestamp) + " s"
    duration_str = "Duration:" + str(duration) + " s"

    print("-" * 100)
    print("Summary Info:")
    print(start_time_str)
    print(end_time_str)
    print(duration_str)
    info_strs.append("-" * 100 + "\n")
    info_strs.append("Summary Info:\n")
    info_strs.append(start_time_str + "\n")
    info_strs.append(end_time_str + "\n")
    info_strs.append(duration_str + "\n")

    return start_timestamp

if __name__ == '__main__':
    imu_path = "/home/xuhui/omni-record/imu_data.txt"
    bag_path = "/home/xuhui/omni-record/2023-02-19-23-16-29.bag"
    out_path = "/home/xuhui/omni-record/2023-02-19-23-16-29-out.bag"
    left_img_name = "/rgb_left"
    right_img_name = "/rgb_right"
    pose_name = "/odom"
    imu_name = "/imu"

    left_imgs, left_timestamps = loadTopicImgs(bag_path, left_img_name)
    right_imgs, right_timestamps = loadTopicImgs(bag_path, right_img_name)
    poses, pose_timestamps = loadOdometry(bag_path, pose_name)
    imu_msgs, imu_timestamps = loadIMU(imu_path)

    print(len(imu_msgs), len(imu_timestamps))

    bag_out = rosbag.Bag(out_path,'w')

    for i in range(len(imu_msgs)):
        bag_out.write(imu_name,imu_msgs[i], imu_timestamps[i])
        print('save imu:',i+1,'/',len(imu_msgs))

    for i in range(len(left_imgs)):
        bag_out.write(left_img_name, left_imgs[i], left_timestamps[i])
        bag_out.write(right_img_name, right_imgs[i], right_timestamps[i])
        print("save img",i+1,"/",len(left_imgs))
    
    for i in range(len(poses)):
        bag_out.write(pose_name, poses[i], pose_timestamps[i])
        print("save pose gt",i+1,"/",len(poses))

    bag_out.close()
