#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin"

tx_index = 5

## 数据准备
## body frame运动时位姿的真值
position = []
position = np.loadtxt(filepath + '/imu_pose.txt', usecols = (tx_index, tx_index+1, tx_index+2))

## 轨迹真值
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2], linewidth=1.0, color="black", label='trajectory_groundtruth')
ax.plot([position[0, 0]], [position[0, 1]], [position[0, 2]], 'o', markersize=3, color="red", label='start point')
ax.legend(loc='upper right', fontsize=9, edgecolor='black')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

## read data from all_points.txt
x=[]
y=[]
z=[]
with open(filepath + '/all_points.txt', 'r') as f:
    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = map(float, odom)  # 转化为浮点数
        x.append(numbers_float[0])
        y.append(numbers_float[1])
        z.append(numbers_float[2])

ax.scatter(x, y, z, color='green')

## 将所有特征点中的其中18个特征点连线，形成house的形状
s = filepath + '/house_model/house.txt'
with open(s, 'r') as f:
    data = f.readlines()
    for line in data:
        odom = line.split()
        numbers_float = map(float, odom)
        ax.plot([numbers_float[0], numbers_float[3]], [numbers_float[1], numbers_float[4]], color='blue', zs=[numbers_float[2], numbers_float[5]])

plt.show()
