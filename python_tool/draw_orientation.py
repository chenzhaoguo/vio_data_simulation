#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin/"


## euler groundtruth  &&  euler tranfer: euler gt --> matrix --> q --> euler
euler_gt = []
euler_x = np.loadtxt(filepath + 'imu_euler_gt.txt', usecols=(0))
euler_gt = np.loadtxt(filepath + 'imu_euler_gt.txt', usecols=(1, 2, 3))
euler_transfer = []
euler_transfer = np.loadtxt(filepath + 'imu_pose.txt', usecols=(14, 15, 16))
diff_euler_transfer_rad = np.array(euler_transfer) - np.array(euler_gt)
diff_euler_transfer_deg = diff_euler_transfer_rad * 180 / math.pi

fig1, (ax11, ax12, ax13) = plt.subplots(3, 1, figsize=(6, 6))
## euler_gt
ax11.plot(euler_x, euler_gt[:, 0], linewidth=1.0, color='r', label='roll')
ax11.plot(euler_x, euler_gt[:, 1], linewidth=1.0, color='g', label='pitch')
ax11.plot(euler_x, euler_gt[:, 2], linewidth=1.0, color='b', label='yaw')
ax11.set_ylabel('euler_gt [rad]', fontsize=10)
ax11.set_title('euler_gt && euler_tranfer', fontsize=12)
ax11.tick_params(labelsize=8)
ax11.legend(loc='upper right', fontsize=6, edgecolor='w')
ax11.grid(linestyle="--")
## euler_transfer
ax12.plot(euler_x, euler_transfer[:, 0], linewidth=1.0, color='r', label='roll')
ax12.plot(euler_x, euler_transfer[:, 1], linewidth=1.0, color='g', label='pitch')
ax12.plot(euler_x, euler_transfer[:, 2], linewidth=1.0, color='b', label='yaw')
ax12.set_ylabel('euler_transfer [rad]', fontsize=10)
ax12.tick_params(labelsize=8)
ax12.legend(loc='upper right', fontsize=6, edgecolor='2')
ax12.grid(linestyle="--")
## diff_euler_transfer_deg
ax13.plot(euler_x, diff_euler_transfer_deg[:, 0], linewidth=1.0, color='r', label='roll')
ax13.plot(euler_x, diff_euler_transfer_deg[:, 1], linewidth=1.0, color='g', label='pitch')
ax13.plot(euler_x, diff_euler_transfer_deg[:, 2], linewidth=1.0, color='b', label='yaw')
ax13.set_xlabel('t [s]', fontsize=10)
ax13.set_ylabel('Orientation error [deg]', fontsize=10)
ax13.tick_params(labelsize=8)
ax13.legend(loc='upper right', fontsize=6, edgecolor='w')
ax13.grid(linestyle="--")


## imu data integration: no noise  &&  diff_euler_int_no_noise_deg
euler_int_no_noise = []
euler_int_no_noise_x = np.loadtxt(filepath + 'imu_int_pose.txt', usecols=(0))
euler_int_no_noise = np.loadtxt(filepath + 'imu_int_pose.txt', usecols=(8, 9, 10))
diff_euler_int_no_noise_rad = np.array(euler_int_no_noise) - np.array(euler_gt[1:18204])
diff_euler_int_no_noise_deg = diff_euler_int_no_noise_rad * 180 / math.pi

fig2, (ax21, ax22) = plt.subplots(2, 1, figsize=(6, 4))
## euler_int_no_noise
ax21.plot(euler_int_no_noise_x, euler_int_no_noise[:, 0], linewidth=1.0, color='r', label='roll')
ax21.plot(euler_int_no_noise_x, euler_int_no_noise[:, 1], linewidth=1.0, color='g', label='pitch')
ax21.plot(euler_int_no_noise_x, euler_int_no_noise[:, 2], linewidth=1.0, color='b', label='yaw')
ax21.set_ylabel('euler_int_no_noise [rad]', fontsize=10)
ax21.tick_params(labelsize=8)
ax21.set_title('euler_int_no_noise && diff_euler_int_no_noise_deg', fontsize=12)
ax21.legend(loc='upper right', fontsize=6, edgecolor='w')
ax21.grid(linestyle="--")
## diff_euler_int_no_noise_deg
ax22.plot(euler_int_no_noise_x, diff_euler_int_no_noise_deg[:, 0], linewidth=1.0, color='r', label='roll')
ax22.plot(euler_int_no_noise_x, diff_euler_int_no_noise_deg[:, 1], linewidth=1.0, color='g', label='pitch')
ax22.plot(euler_int_no_noise_x, diff_euler_int_no_noise_deg[:, 2], linewidth=1.0, color='b', label='yaw')
ax22.set_xlabel('t [s]', fontsize=10)
ax22.set_ylabel('Orientation error [deg]', fontsize=10)
ax22.tick_params(labelsize=8)
ax22.legend(loc='upper right', fontsize=6, edgecolor='w')
ax22.grid(linestyle="--")
## zoom out diff_euler_int_no_noise_deg
insert_ax1 = fig2.add_axes([0.65, 0.2, 0.15, 0.15])
insert_ax1.plot(euler_int_no_noise_x, diff_euler_int_no_noise_deg[:, 0], linewidth=0.8, color='r', label='roll')
insert_ax1.plot(euler_int_no_noise_x, diff_euler_int_no_noise_deg[:, 1], linewidth=0.8, color='g', label='pitch')
insert_ax1.plot(euler_int_no_noise_x, diff_euler_int_no_noise_deg[:, 2], linewidth=0.8, color='b', label='yaw')
insert_ax1.set_xlim(0.005, 20.0)
insert_ax1.set_ylim(-0.01, 0.01)
insert_ax1.tick_params(labelsize=7)
insert_ax1.grid(linestyle="--")


## imu data integration: noise  &&  diff_euler_int_noise_deg
euler_int_noise = []
euler_int_noise_x = np.loadtxt(filepath + 'imu_int_pose_noise.txt', usecols=(0))
euler_int_noise = np.loadtxt(filepath + 'imu_int_pose_noise.txt', usecols = (8, 9 ,10))
diff_euler_int_noise_rad = np.array(euler_int_noise) - np.array(euler_gt[1:18204])
diff_euler_int_noise_deg = diff_euler_int_noise_rad * 180 / math.pi

fig3, (ax31, ax32) = plt.subplots(2, 1, figsize=(6, 4))
## imu data integration: noise
ax31.plot(euler_int_noise_x, euler_int_noise[:, 0], linewidth=1.0, color='r', label='roll')
ax31.plot(euler_int_noise_x, euler_int_noise[:, 1], linewidth=1.0, color='g', label='pitch')
ax31.plot(euler_int_noise_x, euler_int_noise[:, 2], linewidth=1.0, color='b', label='yaw')
ax31.set_ylabel('euler_int_noise [rad]', fontsize=10)
ax31.tick_params(labelsize=8)
ax31.set_title('euler_int_noise && diff_euler_int_noise_deg', fontsize=12)
ax31.legend(loc='upper right', fontsize=6, edgecolor='w')
ax31.grid(linestyle="--")
## diff_euler_int_noise_deg
ax32.plot(euler_int_noise_x, diff_euler_int_noise_deg[:, 0], linewidth=1.0, color='r', label='roll')
ax32.plot(euler_int_noise_x, diff_euler_int_noise_deg[:, 1], linewidth=1.0, color='g', label='pitch')
ax32.plot(euler_int_noise_x, diff_euler_int_noise_deg[:, 2], linewidth=1.0, color='b', label='yaw')
ax32.set_xlabel('t [s]', fontsize=10)
ax32.set_ylabel('Orientation error [deg]', fontsize=10)
ax32.tick_params(labelsize=8)
ax32.legend(loc='upper right', fontsize=6, edgecolor='w')
ax32.grid(linestyle="--")
## zoom out diff_euler_int_noise_deg
insert_ax2 = fig3.add_axes([0.65, 0.2, 0.15, 0.15])
insert_ax2.plot(euler_int_noise_x, diff_euler_int_noise_deg[:, 0], linewidth=0.8, color='r', label='roll')
insert_ax2.plot(euler_int_noise_x, diff_euler_int_noise_deg[:, 1], linewidth=0.8, color='g', label='pitch')
insert_ax2.plot(euler_int_noise_x, diff_euler_int_noise_deg[:, 2], linewidth=0.8, color='b', label='yaw')
insert_ax2.set_xlim(0.005, 20.0)
insert_ax2.set_ylim(-1.0, 1.0)
insert_ax2.tick_params(labelsize=7)
insert_ax2.grid(linestyle="--")


plt.show()
