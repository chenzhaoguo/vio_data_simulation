#ifndef IMUSIMWITHPOINTLINE_UTILITIES_H
#define IMUSIMWITHPOINTLINE_UTILITIES_H

#include "imu.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>

Eigen::Vector3d Quaterniond2EulerAngle(const Eigen::Quaterniond &q);  // [roll, pitch, yaw]

/// save 3d points to file
void SavePoints(std::string filename, 
                 std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points);

/// save 3d points && it's observe in 相机归一化平面
void SaveFeatures(std::string filename,
                  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features);

/// save line observe
void SaveLines(std::string filename,
               std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features);

void LoadPose(std::string filename, std::vector<MotionData> &pose);

/// save imu data
void SaveDataImu(std::string filename, std::vector<MotionData> pose);

/// save camera data
void SaveDataCamera(std::string filename, std::vector<MotionData> pose);

/// save camera data as TUM style
void SaveDataCameraAsTUM(std::string filename, std::vector<MotionData> pose);

/// save euler_angle
void SaveEulerAngle(std::string filename, std::map<double, Eigen::Vector3d> &euler_angles);

#endif  // IMUSIMWITHPOINTLINE_UTILITIES_H