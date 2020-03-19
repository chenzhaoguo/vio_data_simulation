#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin"

col_index = 0

acc_bias = []
acc_x = np.loadtxt(filepath + '/acc_bias.txt', usecols = (col_index))
acc_bias = np.loadtxt(filepath + '/acc_bias.txt', usecols = (col_index+1, col_index+2, col_index+3))

gyro_bias = []
gyro_x = np.loadtxt(filepath + '/gyro_bias.txt', usecols = (col_index))
gyro_bias = np.loadtxt(filepath + '/gyro_bias.txt', usecols = (col_index+1, col_index+2, col_index+3))

## 画图
## acc bias
fig = plt.figure(1)
plt.plot(acc_x, acc_bias[:, 0], linewidth=1.0, color="red", label='X')
plt.plot(acc_x, acc_bias[:, 1], linewidth=1.0, color="green", label='Y')
plt.plot(acc_x, acc_bias[:, 2], linewidth=1.0, color="blue", label='Z')
plt.xlabel('time [s]')
plt.ylabel('acc_bias [rad/s]')
plt.legend(loc='upper right', fontsize=12, edgecolor='black')
plt.grid(linestyle="--")

## gyro bias
fig = plt.figure(2)
plt.plot(gyro_x, gyro_bias[:, 0], linewidth=1.0, color="red", label='X')
plt.plot(gyro_x, gyro_bias[:, 1], linewidth=1.0, color="green", label='Y')
plt.plot(gyro_x, gyro_bias[:, 2], linewidth=1.0, color="blue", label='Z')
plt.xlabel('time [s]')
plt.ylabel('gyro_bias [rad/s]')
plt.legend(loc='upper right', fontsize=12, edgecolor='black')
plt.grid(linestyle="--")

plt.show()
