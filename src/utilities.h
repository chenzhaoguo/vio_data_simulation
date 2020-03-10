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
void save_points(std::string filename, 
                 std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points);

/// save 3d points and it's obs in image
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features);

/// save line obs
void save_lines(std::string filename,
                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features);

void LoadPose(std::string filename, std::vector<MotionData> &pose);

/// save imu body data
void save_Pose(std::string filename, std::vector<MotionData> pose);

/// save pose as TUM style
void save_Pose_asTUM(std::string filename, std::vector<MotionData> pose);

/// save euler_angle
void save_euler_angle(std::string filename, std::map<double, Eigen::Vector3d> &euler_angles);

#endif  // IMUSIMWITHPOINTLINE_UTILITIES_H