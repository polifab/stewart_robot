#! /usr/bin/env python

import rospy
import rosbag
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
import sys
import os
import json
import copy
import numpy as np
from math import isinf
from random import random
# from pyquaternion import Quaternion
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from boostrtrees import RTree
# rv1 = np.array([0.,0.,0.])
# q1  = Quaternion(1,0,0,0)
import pickle

def print_perc(perc, N, i):
    
    if round(i*100./N) != perc  and int(round(i*100./N)) != 100:
        perc = round(i*100./N)
        b = str(">> "+str(int(perc)))+"%"  
        #print (b, end="\r")
        sys.stdout.write("\rProgress "+b)
        sys.stdout.flush()
    elif round(i*100./N) != perc and int(round(i*100./N)) == 100:
        print("\rProgress >> 100%\n")
        perc = round(i*100./N)

    return perc

def convert_quats_eulers(pose_array):
    for i in range(0,len(pose_array)):
        quaternion = pose_array[i,3:]
        rot = R.from_quat(quaternion)
        rot_euler = rot.as_euler('xyz', degrees=True)
        # if np.abs(rot_euler[0] - 180) < 0.1 or np.abs(rot_euler[0] + 180) < 0.1:
        if rot_euler[0] > 170: 
            rot_euler[0] = rot_euler[0] - 180
        if rot_euler[0] < -170:
            rot_euler[0] = rot_euler[0] + 180
        pose_array[i,3:6] = rot_euler
    return pose_array

def convert_eulers_to_quat(traj_array):
    return_array = np.zeros([len(traj_array), 7])
    return_array[:,:3] = traj_array[:,:3]
    for i in range(0,len(traj_array)):
        eulers = traj_array[i,3:]
        rot = R.from_euler('xyz', eulers, degrees=True)
        quaternion = rot.as_quat()
        return_array[i,3:] = quaternion
        # if np.abs(rot_euler[0] - 180) < 0.1 or np.abs(rot_euler[0] + 180) < 0.1:

    return return_array

def read_rosbag():
    
    input_bag_path = rosbags_path

    bag=rosbag.Bag(input_bag_path)

    count_msg=bag.get_message_count()
    counter=0
    perc=0

    trajectories = np.array([0,0,0,0,0,0,0])
    poses = np.array([0,0,0,1,0,0,0]) 
    velocities = np.array([0,0,0,0,0,0])
    time_traj = np.array([])
    time_poses = np.array([])
    time_vels = np.array([])

    trapz_setpoint = None
    for topic, msg, t in bag.read_messages():
        if topic == "/stewart_controller_node/cmd_pos":
            trajectories = np.vstack((trajectories, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]))
            time_traj = np.append(time_traj, msg.header.stamp.to_sec())
        elif topic == "/stewart_controller_node/pose_base": # nav_msgs/Odometry
            poses = np.vstack((poses, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]))
            time_poses = np.append(time_poses, msg.header.stamp.to_sec())
        elif topic == "/stewart_controller_node/pose_vel":
            velocities = np.vstack((velocities, [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]))
            time_vels = np.append(time_vels, msg.header.stamp.to_sec())
        elif topic == "/stewart_controller_node/pose_trapz_setpoint":
            trapz_setpoint = msg

            perc=print_perc(perc,count_msg,counter)
            counter+=1

    setpoint_pos = np.zeros([len(poses)-1,3]) + [trapz_setpoint.position.x, trapz_setpoint.position.y, trapz_setpoint.position.z]
    setpoint_att = np.zeros([len(poses),4]) + [trapz_setpoint.orientation.w, trapz_setpoint.orientation.x, trapz_setpoint.orientation.y, trapz_setpoint.orientation.z]
    print(time_poses)
    print(time_traj)
    fig, axs = plt.subplots(2, sharex=True)
    fig.suptitle("Position Trapezoidal Trajectories")
    axs[0].set_title("Positions")
    axs[0].plot(time_poses, poses[1:,0], 'b',label="x")
    axs[0].plot(time_poses, poses[1:,1], 'g',label="y")
    axs[0].plot(time_poses, poses[1:,2], 'r',label="z")

    axs[0].plot(time_traj, trajectories[1:,0], color="#3366cc", linestyle='-.', label="plan x")
    axs[0].plot(time_traj, trajectories[1:,1], color="#339933", linestyle='-.', label="plan y")
    axs[0].plot(time_traj, trajectories[1:,2], color="#ff5050", linestyle='-.', label="plan z")

    axs[0].plot(time_poses, setpoint_pos[:,0], color="#003366", linestyle='dashed', label="setpoint_x")
    axs[0].plot(time_poses, setpoint_pos[:,1], color="#003300", linestyle='dashed', label="setpoint_y")
    axs[0].plot(time_poses, setpoint_pos[:,2], color="#800000", linestyle='dashed', label="setpoint_z")
    
    axs[0].grid()
    axs[0].legend()
    axs[0].set(ylabel='$m$')
    axs[0].set(xlabel='s')

    axs[1].set_title("Linear Velocities")
    axs[1].plot(time_vels, velocities[1:,0], 'b', label="x")
    axs[1].plot(time_vels, velocities[1:,1], 'g', label="y")
    axs[1].plot(time_vels, velocities[1:,2], 'r', label="z")

    axs[1].grid()
    axs[1].legend()
    axs[1].set(ylabel='$m \cdot s^{-1}$')
    axs[1].set(xlabel='s')

    plt.show()

    fig, axs = plt.subplots(2, sharex=True)
    fig.suptitle("Orientation Trapezoidal Trajectories")
    axs[0].set_title("Attitude")

    # axs[0].plot(time_poses, poses[1:,3], 'b',label="w")
    axs[0].plot(time_poses, poses[1:,4], 'b',label="x")
    axs[0].plot(time_poses, poses[1:,5], 'g',label="y")
    axs[0].plot(time_poses, poses[1:,6], 'r',label="z")

    axs[0].plot(time_traj, trajectories[1:,4], color="#3366cc", linestyle='-.', label="plan x")
    axs[0].plot(time_traj, trajectories[1:,5], color="#339933", linestyle='-.', label="plan y")
    axs[0].plot(time_traj, trajectories[1:,6], color="#ff5050", linestyle='-.', label="plan z")
    # axs[0].plot(time_traj, new_traj[1:,6], color="#ff5050", linestyle='-.', label="plan z")

    axs[0].plot(time_poses, setpoint_att[1:,1], color="#003366", linestyle='dashed', label="setpoint_x")
    axs[0].plot(time_poses, setpoint_att[1:,2], color="#003300", linestyle='dashed', label="setpoint_y")
    axs[0].plot(time_poses, setpoint_att[1:,3], color="#800000", linestyle='dashed', label="setpoint_z")
    
    axs[0].grid()
    axs[0].legend()
    axs[0].set(ylabel='$quaternion$')
    axs[0].set(xlabel='s')

    axs[1].set_title("Angular Velocities")
    axs[1].plot(time_vels, velocities[1:,3], 'b', label="x")
    axs[1].plot(time_vels, velocities[1:,4], 'g', label="y")
    axs[1].plot(time_vels, velocities[1:,5], 'r', label="z")

    axs[1].grid()
    axs[1].legend()
    axs[1].set(ylabel='$rad \cdot s^{-1}$')
    axs[1].set(xlabel='s')

    plt.show()


    print("------------")


if __name__ == '__main__':

    rosbags_path = sys.argv[1]  # Rosbag path

    read_rosbag()