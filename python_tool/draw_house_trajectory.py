#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin/"

## body frame运动时位姿的真值
position = []
position = np.loadtxt(filepath + 'imu_pose.txt', usecols = (5, 6, 7))
## plot trajectory groundtruth
fig = plt.figure(num=1, figsize=(8, 6))
ax = fig.gca(projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2], linewidth=1.0, color='k', label='trajectory_gt')
ax.plot([position[0, 0]], [position[0, 1]], [position[0, 2]], 'o', markersize=4, color='r', label='start point')
ax.set_xlabel('X [m]', fontsize=10)
ax.set_ylabel('Y [m]', fontsize=10)
ax.set_zlabel('Z [m]', fontsize=10)
ax.tick_params(labelsize=9)
ax.legend(loc='upper right', fontsize=8, edgecolor='k')
# ax.view_init(elev=40.0, azim=-20)


## read data from all_points.txt
x=[]
y=[]
z=[]
with open(filepath + 'all_points.txt', 'r') as f:
    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数
        x.append(numbers_float[0])
        y.append(numbers_float[1])
        z.append(numbers_float[2])
ax.scatter(x, y, z, s=12, color='g')

## 将所有特征点中的其中18个特征点连线，形成house的形状
s = filepath + '../landmarks_data/auditorium.txt'
with open(s, 'r') as f:
    data = f.readlines()
    for line in data:
        odom = line.split()
        numbers_float = map(float, odom)
        ax.plot([numbers_float[0], numbers_float[3]], [numbers_float[1], numbers_float[4]], [numbers_float[2], numbers_float[5]], linestyle='--', linewidth=1.0, color='b')

plt.show()
