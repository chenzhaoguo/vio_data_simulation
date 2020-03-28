#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..') + "/bin/"

## load data
acc_bias = []
acc_bias = np.loadtxt(filepath + 'acc_bias_gt.txt', usecols=(0, 1, 2, 3))
gyro_bias = []
gyro_bias = np.loadtxt(filepath + 'gyro_bias_gt.txt', usecols=(0, 1, 2, 3))

## acc bias
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 4))
ax1.plot(acc_bias[:, 0], acc_bias[:, 1], linewidth=0.8, color='r', label='X')
ax1.plot(acc_bias[:, 0], acc_bias[:, 2], linewidth=0.8, color='g', label='Y')
ax1.plot(acc_bias[:, 0], acc_bias[:, 3], linewidth=0.8, color='b', label='Z')
ax1.set_ylabel('acc_bias [m/s^2]', fontsize=10)
ax1.tick_params(labelsize=8)
ax1.legend(loc='upper right', fontsize=8, edgecolor='w')
ax1.grid(linestyle="--")

## gyro bias
ax2.plot(gyro_bias[:, 0], gyro_bias[:, 1], linewidth=0.8, color='r', label='X')
ax2.plot(gyro_bias[:, 0], gyro_bias[:, 2], linewidth=0.8, color='g', label='Y')
ax2.plot(gyro_bias[:, 0], gyro_bias[:, 3], linewidth=0.8, color='b', label='Z')
ax2.set_xlabel('t [s]', fontsize=10)
ax2.set_ylabel('gyro_bias [rad/s^3]', fontsize=10)
ax2.tick_params(labelsize=8)
ax2.legend(loc='upper right', fontsize=8, edgecolor='w')
ax2.grid(linestyle="--")

plt.show()
