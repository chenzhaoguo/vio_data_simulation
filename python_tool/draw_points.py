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
from GeometryLib import drawCoordinateFrame, euler2Rbn, euler2Rnb
import transformations as tf

filepath=os.path.abspath('..') + "/bin"

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

## read data from cam_pose.txt
position = []
quaterntions = []
timestamp = []
qw_index = 1
with open(filepath + '/cam_pose.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        odom = line.split()
        numbers_float = map(float, odom)
        #timestamp.append(numbers_float[0])
        quaterntions.append([numbers_float[qw_index], numbers_float[qw_index+1], numbers_float[qw_index+2], numbers_float[qw_index+3]])  # qw,qx,qy,qz
        position.append([numbers_float[qw_index+4], numbers_float[qw_index+5], numbers_float[qw_index+6]])


## plot 3d
fig = plt.figure()
plt.ion()
ax = fig.gca(projection='3d')
rpy = []
t = []
for i in range(0,600,5):
    ax.clear()
    ## 绘制all_points.txt中所有特征点
    ax.scatter(x, y, z, color='green')
    
    ## 将所有特征点中的其中18个特征点连线，形成house的形状
    s = filepath + '/house_model/house.txt'
    with open(s, 'r') as f:
        data = f.readlines()
        for line in data:
            odom = line.split()
            numbers_float = map(float, odom)
            ax.plot([numbers_float[0], numbers_float[3]], [numbers_float[1], numbers_float[4]], color='blue', zs=[numbers_float[2], numbers_float[5]])

    ## 绘制所有时刻相机的位姿，每个位姿用3维坐标系表示
    x1=[]
    y1=[]
    z1=[]
    rpy.append(tf.euler_from_quaternion(quaterntions[i]))
    t.append(position[i])
    for j in range(len(rpy)):
        drawCoordinateFrame(ax, rpy[j], t[j])
    
    ## 绘制当前相机能看到的所有特征点与当前相机位姿坐标系原点的连线
    s = filepath + '/keyframe/all_points_' + str(i) + '.txt'
    p = position[i]
    with open(s, 'r') as f:
        data = f.readlines()
        for line in data:
            odom = line.split()
            numbers_float = map(float, odom)
            x1.append(numbers_float[0])
            y1.append(numbers_float[1])
            z1.append(numbers_float[2])
            ax.plot([numbers_float[0], p[0]], [numbers_float[1], p[1]], zs=[numbers_float[2], p[2]])
    
    ax.scatter(x1, y1, z1, color='red', marker='^')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.set_xlim(-15, 20)
    # ax.set_ylim(-15, 20)
    # ax.set_zlim(0, 20)
    ax.legend()
    plt.show()
    plt.pause(0.01)
