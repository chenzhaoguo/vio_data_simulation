#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin/"

## load data
pose_gt = []
pose_gt = np.loadtxt(filepath + 'imu_pose.txt', usecols = (5, 6, 7))
pose_int_no_noise = []
pose_int_no_noise = np.loadtxt(filepath + 'imu_int_pose.txt', usecols = (5, 6, 7))
pose_int_noise = []
pose_int_noise = np.loadtxt(filepath + 'imu_int_pose_noise.txt', usecols = (5, 6, 7))
## position error
diff_x = np.loadtxt(filepath + 'imu_int_pose.txt', usecols = (0))
diff_position_no_noise = np.array(pose_int_no_noise) - np.array(pose_gt[1:4001])
diff_position_noise = np.array(pose_int_noise) - np.array(pose_gt[1:4001])


## trajectory with imu(no noise)
fig1 = plt.figure(num=1)
ax1 = fig1.gca(projection='3d')
ax1.plot(pose_gt[:, 0], pose_gt[:, 1], pose_gt[:, 2], linewidth=1.0, color='k', label='groundtruth')
ax1.plot(pose_int_no_noise[:, 0], pose_int_no_noise[:, 1], pose_int_no_noise[:, 2], linewidth=1.0, color='b', label='imu_int_no_noise', linestyle='--')
ax1.plot([pose_gt[0, 0]], [pose_gt[0, 1]], [pose_gt[0, 2]], 'o', markersize=3, color='r', label='start point')
ax1.set_xlabel('X', fontsize=10)
ax1.set_ylabel('Y', fontsize=10)
ax1.set_zlabel('Z', fontsize=10)
ax1.tick_params(labelsize=8)
ax1.legend(loc='upper right', fontsize=8, edgecolor='w')

## trajectory error with imu(no noise)
fig2, ax2 = plt.subplots(num=2, figsize=(6, 4))
ax2.plot(diff_x, diff_position_no_noise[:, 0], linewidth=1.0, color='r', label='X')
ax2.plot(diff_x, diff_position_no_noise[:, 1], linewidth=1.0, color='g', label='Y')
ax2.plot(diff_x, diff_position_no_noise[:, 2], linewidth=1.0, color='b', label='Z')
ax2.set_xlabel('t [s]', fontsize=10)
ax2.set_ylabel('Position error [m]', fontsize=10)
ax2.tick_params(labelsize=8)
ax2.legend(loc='upper right', fontsize=8, edgecolor='w')
ax2.grid(linestyle="--")


## trajectory with imu(noise)
fig3 = plt.figure(num=3)
ax3 = fig3.gca(projection='3d')
ax3.plot(pose_gt[:, 0], pose_gt[:, 1], pose_gt[:, 2], linewidth=1.0, color='k', label='groundtruth')
ax3.plot(pose_int_noise[:, 0], pose_int_noise[:, 1], pose_int_noise[:, 2], linewidth=1.0, color='b', label='imu_int_noise', linestyle='--')
ax3.plot([pose_gt[0, 0]], [pose_gt[0, 1]], [pose_gt[0, 2]], 'o', markersize=3, color='r', label='start point')
ax3.set_xlabel('X', fontsize=10)
ax3.set_ylabel('Y', fontsize=10)
ax3.set_zlabel('Z', fontsize=10)
ax3.tick_params(labelsize=8)
ax3.legend(loc='upper right', fontsize=8, edgecolor='w')

## trajectory error with imu(noise)
fig4, ax4 = plt.subplots(num=4, figsize=(6, 4))
ax4.plot(diff_x, diff_position_noise[:, 0], linewidth=1.0, color='r', label='X')
ax4.plot(diff_x, diff_position_noise[:, 1], linewidth=1.0, color='g', label='Y')
ax4.plot(diff_x, diff_position_noise[:, 2], linewidth=1.0, color='b', label='Z')
ax4.set_xlabel('t [s]', fontsize=10)
ax4.set_ylabel('Position error [m]', fontsize=10)
ax4.tick_params(labelsize=8)
ax4.legend(loc='upper right', fontsize=8, edgecolor='w')
ax4.grid(linestyle="--")

plt.show()
