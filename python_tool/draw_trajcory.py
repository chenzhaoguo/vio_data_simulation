#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 18:18:24 2017

@author: hyj
"""
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..')+"/bin"

tx_index = 5

## 数据准备
## body frame运动时位姿的真值
position = []
position = np.loadtxt(filepath + '/imu_pose.txt', usecols = (tx_index, tx_index+1, tx_index+2))
## 无噪声的IMU数据积分得到的body frame的位姿
position1 = []
position1 = np.loadtxt(filepath + '/imu_int_pose.txt', usecols = (tx_index, tx_index+1, tx_index+2))
## 有噪声的IMU数据积分得到的body frame的位姿
position2 = []
position2 = np.loadtxt(filepath + '/imu_int_pose_noise.txt', usecols = (tx_index, tx_index+1, tx_index+2))
## position error
position_gt = position[1:4001]
diff_position_no_noise = np.array(position1) - np.array(position_gt)
diff_position_noise = np.array(position2) - np.array(position_gt)

## 画图
## 无噪声IMU数据积分解算的轨迹与真值比较
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2], linewidth=1.0, color="black", label='groundtruth')
ax.plot(position1[:, 0], position1[:, 1], position1[:, 2], linewidth=1.0, color="blue", label='imu_integration_no_noise', linestyle='--')
ax.plot([position[0, 0]], [position[0, 1]], [position[0, 2]], 'o', markersize=3, color="red", label='start point')
ax.legend(loc='upper right', fontsize=9, edgecolor='black')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# 画position误差图
fig = plt.figure(2)
x = np.linspace(0.005, 20, 4000)
plt.plot(x, diff_position_no_noise[:, 0], linewidth=1.0, color="red", label='X')
plt.plot(x, diff_position_no_noise[:, 1], linewidth=1.0, color="green", label='Y')
plt.plot(x, diff_position_no_noise[:, 2], linewidth=1.0, color="blue", label='Z')
plt.xlabel('time [s]')
plt.ylabel('Position error [m]')
plt.legend(loc='upper right', fontsize=12, edgecolor='black')
plt.grid(linestyle="--")

# ## 有噪声IMU数据积分解算的轨迹与真值比较
fig = plt.figure(3)
ax = fig.gca(projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2], linewidth=1.0, color="black", label='groundtruth')
ax.plot(position2[:, 0], position2[:, 1], position2[:, 2], linewidth=1.0, color="blue", label='imu_integration_noise', linestyle='--')
ax.plot([position[0, 0]], [position[0, 1]], [position[0, 2]], 'o', markersize=3, color="red", label='start point')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend(loc='upper right', fontsize=9, edgecolor='black')
## 画position误差图
fig = plt.figure(4)
x = np.linspace(0.005, 20.0, 4000)
plt.plot(x, diff_position_noise[:, 0], linewidth=1.0, color="red", label='X')
plt.plot(x, diff_position_noise[:, 1], linewidth=1.0, color="green", label='Y')
plt.plot(x, diff_position_noise[:, 2], linewidth=1.0, color="blue", label='Z')
plt.xlabel('time [s]')
plt.ylabel('Position error [m]')
plt.legend(loc='upper right', fontsize=12, edgecolor='black')
plt.grid(linestyle="--")

plt.show()
