#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin"

tx_index = 1

## euler groundtruth
euler_gt = []
euler_gt = np.loadtxt(filepath + '/imu_euler_gt.txt', usecols = (tx_index, tx_index+1, tx_index+2))
## 画图
# fig = plt.figure(1)
# x = np.linspace(0.0, 20.0, 4001)
# plt.plot(x, euler_gt[:, 0], linewidth=1.0, color="red", label='roll')
# plt.plot(x, euler_gt[:, 1], linewidth=1.0, color="green", label='pitch')
# plt.plot(x, euler_gt[:, 2], linewidth=1.0, color="blue", label='yaw')
# plt.legend(loc='upper right', fontsize=12, edgecolor='black')
# plt.xlabel('time [s]')
# plt.ylabel('euler angle [rad]')
# plt.grid(linestyle="--")

## euler gt --> matrix --> q --> euler
euler_transfer = []
euler_transfer = np.loadtxt(filepath + '/imu_pose.txt', usecols = (tx_index+13, tx_index+14, tx_index+15))
## 画图
# fig = plt.figure(2)
# x = np.linspace(0.0, 20.0, 4001)
# plt.plot(x, euler_transfer[:, 0], linewidth=1.0, color="red", label='roll')
# plt.plot(x, euler_transfer[:, 1], linewidth=1.0, color="green", label='pitch')
# plt.plot(x, euler_transfer[:, 2], linewidth=1.0, color="blue", label='yaw')
# plt.legend(loc='upper right', fontsize=12, edgecolor='black')
# plt.xlabel('time [s]')
# plt.ylabel('euler angle [rad]')
# plt.grid(linestyle="--")

## imu data integration: no noise
euler_int_no_noise = []
euler_int_no_noise = np.loadtxt(filepath + '/imu_int_pose.txt', usecols = (tx_index+14, tx_index+15, tx_index+16))
## 画图
# fig = plt.figure(3)
# x = np.linspace(0.05, 20.0, 4000)
# plt.plot(x, euler_int_no_noise[:, 0], linewidth=1.0, color="red", label='roll')
# plt.plot(x, euler_int_no_noise[:, 1], linewidth=1.0, color="green", label='pitch')
# plt.plot(x, euler_int_no_noise[:, 2], linewidth=1.0, color="blue", label='yaw')
# plt.legend(loc='upper right', fontsize=12, edgecolor='black')
# plt.xlabel('time [s]')
# plt.ylabel('euler angle [rad]')
# plt.grid(linestyle="--")

## imu data integration: noise
euler_int_noise = []
euler_int_noise = np.loadtxt(filepath + '/imu_int_pose_noise.txt', usecols = (tx_index+14, tx_index+15, tx_index+16))
## 画图
# fig = plt.figure(4)
# x = np.linspace(0.05, 20.0, 4000)
# plt.plot(x, euler_int_noise[:, 0], linewidth=1.0, color="red", label='roll')
# plt.plot(x, euler_int_noise[:, 1], linewidth=1.0, color="green", label='pitch')
# plt.plot(x, euler_int_noise[:, 2], linewidth=1.0, color="blue", label='yaw')
# plt.legend(loc='upper right', fontsize=12, edgecolor='black')
# plt.xlabel('time [s]')
# plt.ylabel('euler angle [rad]')
# plt.grid(linestyle="--")

# diff_euler_transfer_rad = np.array(euler_transfer) - np.array(euler_gt)
# diff_euler_transfer_deg = diff_euler_transfer_rad * 180 / math.pi
## 画图: diff_euler_transfer_deg
# fig = plt.figure(5)
# x = np.linspace(0.0, 20.0, 4001)
# plt.plot(x, diff_euler_transfer_deg[:, 0], linewidth=1.0, color="red", label='roll')
# plt.plot(x, diff_euler_transfer_deg[:, 1], linewidth=1.0, color="green", label='pitch')
# plt.plot(x, diff_euler_transfer_deg[:, 2], linewidth=1.0, color="blue", label='yaw')
# plt.xlabel('time [s]')
# plt.ylabel('Orientation error [deg]')
# plt.legend(loc='upper right', fontsize=12, edgecolor='black')
# plt.grid(linestyle="--")

diff_euler_int_no_noise_rad = np.array(euler_int_no_noise) - np.array(euler_gt[1:4001])
diff_euler_int_no_noise_deg = diff_euler_int_no_noise_rad * 180 / math.pi
## 画图: diff_euler_int_no_noise_deg
fig = plt.figure(6)
x = np.linspace(0.005, 20.0, 4000)
plt.plot(x, diff_euler_int_no_noise_deg[:, 0], linewidth=1.0, color="red", label='roll')
plt.plot(x, diff_euler_int_no_noise_deg[:, 1], linewidth=1.0, color="green", label='pitch')
plt.plot(x, diff_euler_int_no_noise_deg[:, 2], linewidth=1.0, color="blue", label='yaw')
plt.xlabel('time [s]')
plt.ylabel('Orientation error [deg]')
plt.legend(loc='upper right', fontsize=12, edgecolor='black')
plt.grid(linestyle="--")

diff_euler_int_noise_rad = np.array(euler_int_noise) - np.array(euler_gt[1:4001])
diff_euler_int_noise_deg = diff_euler_int_noise_rad * 180 / math.pi
## 画图: diff_euler_int_noise_deg
fig = plt.figure(7)
x = np.linspace(0.005, 20.0, 4000)
plt.plot(x, diff_euler_int_noise_deg[:, 0], linewidth=1.0, color="red", label='roll')
plt.plot(x, diff_euler_int_noise_deg[:, 1], linewidth=1.0, color="green", label='pitch')
plt.plot(x, diff_euler_int_noise_deg[:, 2], linewidth=1.0, color="blue", label='yaw')
plt.xlabel('time [s]')
plt.ylabel('Orientation error [deg]')
plt.legend(loc='upper right', fontsize=12, edgecolor='black')
plt.grid(linestyle="--")

plt.show()
