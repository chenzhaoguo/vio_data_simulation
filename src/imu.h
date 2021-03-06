#ifndef IMUSIMWITHPOINTLINE_IMU_H
#define IMUSIMWITHPOINTLINE_IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <map>
#include "param.h"

struct MotionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  Eigen::Matrix3d Rwb;
  Eigen::Vector3d twb;
  Eigen::Vector3d imu_acc;
  Eigen::Vector3d imu_gyro;

  Eigen::Vector3d imu_gyro_bias;
  Eigen::Vector3d imu_acc_bias;

  Eigen::Vector3d imu_velocity;
};

/// euler2Rotation: body frame to interitail frame
Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles);
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);

class IMU {
 public:
  IMU(Param p);

  MotionData MotionModel(double t);
  void addIMUnoise(MotionData &data);
  
  void TestImu(std::string src, std::string dest);  // imu数据进行积分，用来看imu轨迹

  Param param_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d acc_bias_;

  Eigen::Vector3d init_velocity_;
  Eigen::Vector3d init_twb_;
  Eigen::Matrix3d init_Rwb_;

  /// imu orientation by euler_angle
  std::map<double, Eigen::Vector3d> euler_angles_all_;  // roll/pitch/yaw

  /// save all imu bias
  std::map<double, Eigen::Vector3d> acc_bias_all_;
  std::map<double, Eigen::Vector3d> gyro_bias_all_;
};

#endif  // IMUSIMWITHPOINTLINE_IMU_H