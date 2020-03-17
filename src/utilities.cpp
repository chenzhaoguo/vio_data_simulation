#include "utilities.h"

Eigen::Vector3d Quaterniond2EulerAngle(const Eigen::Quaterniond &q) {
  Eigen::Vector3d euler;  // [roll, pitch, yaw]
  /// roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  euler[0] = atan2(sinr_cosp, cosr_cosp);

  /// pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1) {
    euler[1] = copysign(M_PI/2, sinp); // use 90 degrees if out of range
  } else {
    euler[1] = asin(sinp);
  }

  /// yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  euler[2] = atan2(siny_cosp, cosy_cosp);

  return euler;
}

void SavePoints(std::string filename, 
                 std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points) {
  std::ofstream save_points;
  save_points.open(filename.c_str());

  for (int i = 0; i < points.size(); ++i) {
    Eigen::Vector4d p = points[i];
    save_points << p(0) << " "
                << p(1) << " "
                << p(2) << " "
                << p(3) << std::endl;
  }
}

void SaveFeatures(std::string filename,
                  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features) {
  std::ofstream save_points;
  save_points.open(filename.c_str());

  for (int i = 0; i < points.size(); ++i) {
    Eigen::Vector4d p = points[i];
    Eigen::Vector2d f = features[i];
    save_points << p(0) << " "
                << p(1) << " "
                << p(2) << " "
                << p(3) << " "
                << f(0) << " "
                << f(1) << std::endl;
  }
}

void SaveLines(std::string filename,
               std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features) {
  std::ofstream save_points;
  save_points.open(filename.c_str());

  for (int i = 0; i < features.size(); ++i) {
    Eigen::Vector4d f = features[i];
    save_points << f(0) << " "
                << f(1) << " "
                << f(2) << " "
                << f(3) << std::endl;
  }
}

void LoadPose(std::string filename, std::vector<MotionData> &pose) {
  std::ifstream f;
  f.open(filename.c_str());

  if (!f.is_open()) {
    std::cerr << " can't open LoadFeatures file " << std::endl;
    return;
  }

  while (!f.eof()) {
    std::string s;
    std::getline(f, s);

    if (!s.empty()) {
      std::stringstream ss;
      ss << s;

      double time;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      Eigen::Vector3d gyro;
      Eigen::Vector3d acc;
      ss >> time;
      ss >> q.w();
      ss >> q.x();
      ss >> q.y();
      ss >> q.z();
      ss >> t(0);
      ss >> t(1);
      ss >> t(2);
      ss >> gyro(0);
      ss >> gyro(1);
      ss >> gyro(2);
      ss >> acc(0);
      ss >> acc(1);
      ss >> acc(2);

      MotionData data;
      data.timestamp = time;
      data.imu_gyro = gyro;
      data.imu_acc = acc;
      data.twb = t;
      data.Rwb = Eigen::Matrix3d(q);
      pose.push_back(data);
    }
  }
}

void SaveDataImu(std::string filename, std::vector<MotionData> pose) {
  std::ofstream save_data;
  save_data.open(filename.c_str());

  for (int i = 0; i < pose.size(); ++i) {
    MotionData data = pose[i];
    double time = data.timestamp;
    Eigen::Quaterniond q(data.Rwb);
    Eigen::Vector3d euler = Quaterniond2EulerAngle(q);  // output: roll/pitch/yaw
    Eigen::Vector3d t = data.twb;
    Eigen::Vector3d gyro = data.imu_gyro;
    Eigen::Vector3d acc = data.imu_acc;

    save_data << time << " "
                << q.w() << " "
                << q.x() << " "
                << q.y() << " "
                << q.z() << " "
                << t(0) << " "
                << t(1) << " "
                << t(2) << " "
                << gyro(0) << " "
                << gyro(1) << " "
                << gyro(2) << " "
                << acc(0) << " "
                << acc(1) << " "
                << acc(2) << " "
                << euler(0) << " "
                << euler(1) << " "
                << euler(2) << std::endl;
  }
}

void SaveGroundtruthAsTUM(std::string filename, std::vector<MotionData> pose) {
  std::ofstream save_imu;
  save_imu.setf(std::ios::fixed, std::ios::floatfield);
  save_imu.open(filename.c_str());

  for (int i = 0; i < pose.size(); ++i) {
    MotionData data = pose[i];
    double time = data.timestamp;
    Eigen::Quaterniond q(data.Rwb);
    Eigen::Vector3d t = data.twb;

    save_imu.precision(9);
    save_imu << time << " ";
    save_imu.precision(5);
    save_imu << t(0) << " "
             << t(1) << " "
             << t(2) << " "
             << q.x() << " "
             << q.y() << " "
             << q.z() << " "
             << q.w() << std::endl;
  }
}

void SaveImuOutput(std::string filename, std::vector<MotionData> pose) {
  std::ofstream save_imu;
  save_imu.open(filename.c_str());

  for (int i = 0; i < pose.size(); ++i) {
    MotionData data = pose[i];
    double time = data.timestamp;
    Eigen::Vector3d gyro = data.imu_gyro;
    Eigen::Vector3d acc = data.imu_acc;

    save_imu << time << " "
             << gyro(0) << " "
             << gyro(1) << " "
             << gyro(2) << " "
             << acc(0) << " "
             << acc(1) << " "
             << acc(2) << std::endl;
  }
}

void SaveDataCamera(std::string filename, std::vector<MotionData> pose) {
  std::ofstream save_points;
  save_points.open(filename.c_str());

  for (int i = 0; i < pose.size(); ++i) {
    MotionData data = pose[i];
    double time = data.timestamp;
    Eigen::Quaterniond q(data.Rwb);
    Eigen::Vector3d euler = Quaterniond2EulerAngle(q);  // output: roll/pitch/yaw
    Eigen::Vector3d t = data.twb;

    save_points << time << " "
                << q.w() << " "
                << q.x() << " "
                << q.y() << " "
                << q.z() << " "
                << t(0) << " "
                << t(1) << " "
                << t(2) << " "
                << euler(0) << " "
                << euler(1) << " "
                << euler(2) << std::endl;
  }
}

void SaveDataCameraAsTUM(std::string filename, std::vector<MotionData> pose) {
  std::ofstream save_points;
  save_points.setf(std::ios::fixed, std::ios::floatfield);
  save_points.open(filename.c_str());

  for (int i = 0; i < pose.size(); ++i) {
    MotionData data = pose[i];
    double time = data.timestamp;
    Eigen::Quaterniond q(data.Rwb);
    Eigen::Vector3d t = data.twb;

    save_points.precision(9);
    save_points << time << " ";
    save_points.precision(5);
    save_points << t(0) << " "
                << t(1) << " "
                << t(2) << " "
                << q.x() << " "
                << q.y() << " "
                << q.z() << " "
                << q.w() << std::endl;
  }
}

void SaveEulerAngle(std::string filename, std::map<double, Eigen::Vector3d> &euler_angles) {
  std::ofstream save_euler;
  save_euler.open(filename.c_str());

  for (auto iter = euler_angles.begin(); iter != euler_angles.end(); ++iter) {
    double time = iter->first;
    Eigen::Vector3d euler = iter->second;

    save_euler << time << " "
                << euler(0) << " "
                << euler(1) << " "
                << euler(2) << std::endl;
  }
}