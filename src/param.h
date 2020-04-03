#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <eigen3/Eigen/Core>

class Param {
 public:
  Param();

  /// time
  int imu_frequency = 200;
  int cam_frequency = 30;
  double imu_timestep = 1.0/imu_frequency;
  double cam_timestep = 1.0/cam_frequency;
  double t_start = 0.0;  // s
  double t_end = 40.0;

  /// noise
  double acc_noise_sigma = 2.0e-3;   // 连续时间下acc高斯白噪声方差  m * s^-2 * 1/sqrt(Hz)
  double gyro_noise_sigma = 1.7e-3;  // 连续时间下gyro高斯白噪声方差  rad * s^-1 * 1/sqrt(Hz)
  double acc_bias_sigma = 3.0e-3;  // 连续时间下acc bias随机游走噪声方差  m * s^-3 * 1/sqrt(Hz)
  double gyro_bias_sigma = 2.0e-5;  // 连续时间下gyro bias随机游走噪声方差  rad * s^-2 * 1/sqrt(Hz)

  double pixel_noise = 1;  // 1 pixel noise

  /// camera参数
  double fx = 460;
  double fy = 460;
  double cx = 255;
  double cy = 255;
  double image_w = 640;
  double image_h = 640;

  /// 外参数: camera frame to body frame
  Eigen::Matrix3d R_bc; 
  Eigen::Vector3d t_bc;
};

#endif  // IMUSIM_PARAM_H
