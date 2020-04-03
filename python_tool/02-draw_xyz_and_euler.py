#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from GeometryLib import drawCoordinateFrame, euler2Rbn, euler2Rnb
import transformations as tf

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin/"

## body frame运动时位姿的真值
position_x = []
position = []
position_x = np.loadtxt(filepath + 'imu_pose.txt', usecols = (0))
position = np.loadtxt(filepath + 'imu_pose.txt', usecols = (5, 6, 7))

fig1, (ax11, ax12, ax13) = plt.subplots(3, 1, figsize=(4.5, 3.5), sharex=True)
## x
ax11.plot(position_x, position[:,0], linestyle='-', linewidth=1.0, color='r')
ax11.set_ylabel('x [m]', fontsize=9)
ax11.tick_params(labelsize=7)
ax11.grid(linestyle="--")
## y
ax12.plot(position_x, position[:,1], linestyle='-', linewidth=1.0, color='g')
ax12.set_ylabel('y [m]', fontsize=9)
ax12.tick_params(labelsize=7)
ax12.grid(linestyle="--")
## z
ax13.plot(position_x, position[:,2], linestyle='-', linewidth=1.0, color='b')
ax13.set_xlabel('t [s]', fontsize=9)
ax13.set_ylabel('z [m]', fontsize=9)
ax13.tick_params(labelsize=7)
ax13.grid(linestyle="--")

# fig2, ax2 = plt.subplots(figsize=(6, 3.5))
# ax2.plot(position_x, position[:,0], linestyle='-', linewidth=1.0, color='r', label='x')
# ax2.plot(position_x, position[:,1], linestyle='-', linewidth=1.0, color='g', label='y')
# ax2.plot(position_x, position[:,2], linestyle='-', linewidth=1.0, color='b', label='z')
# ax2.set_xlabel('t [s]', fontsize=10)
# ax2.set_ylabel('position [m]', fontsize=10)
# ax2.tick_params(labelsize=9)
# ax2.legend(loc='upper right', fontsize=6, edgecolor='w')
# ax2.grid(linestyle="--")


## euler angle groundtruth
euler_x = []
euler_gt = []
euler_x = np.loadtxt(filepath + 'imu_euler_gt.txt', usecols = (0))
euler_gt = np.loadtxt(filepath + 'imu_euler_gt.txt', usecols = (1, 2, 3))
euler_gt_deg = euler_gt * 180 / math.pi

fig3, (ax31, ax32, ax33) = plt.subplots(3, 1, figsize=(4.5, 3.5), sharex=True)
## roll
ax31.plot(euler_x, euler_gt_deg[:,0], linestyle='-', linewidth=1.0, color='r')
ax31.set_ylabel('roll [deg]', fontsize=9)
ax31.tick_params(labelsize=7)
ax31.grid(linestyle="--")
## pitch
ax32.plot(euler_x, euler_gt_deg[:,1], linestyle='-', linewidth=1.0, color='g')
ax32.set_ylabel('pitch [deg]', fontsize=9)
ax32.tick_params(labelsize=7)
ax32.grid(linestyle="--")
## yaw
ax33.plot(euler_x, euler_gt_deg[:,2], linestyle='-', linewidth=1.0, color='b')
ax33.set_ylabel('yaw [deg]', fontsize=9)
ax33.set_xlabel('t [s]', fontsize=9)
ax33.tick_params(labelsize=7)
ax33.grid(linestyle="--")

# fig4, ax4 = plt.subplots(figsize=(6, 3.5))
# ax4.plot(euler_x, euler_gt_deg[:,0], linestyle='-', linewidth=1.0, color='r', label='roll')
# ax4.plot(euler_x, euler_gt_deg[:,1], linestyle='-', linewidth=1.0, color='g', label='pitch')
# ax4.plot(euler_x, euler_gt_deg[:,2], linestyle='-', linewidth=1.0, color='b', label='yaw')
# ax4.set_xlabel('t [s]', fontsize=10)
# ax4.set_ylabel('euler angle [deg]', fontsize=10)
# ax4.tick_params(labelsize=9)
# ax4.legend(loc='upper right', fontsize=6, edgecolor='w')
# ax4.grid(linestyle="--")

plt.show()
