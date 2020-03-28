#include <fstream>
#include <sys/stat.h>
#include "../src/imu.h"
#include "../src/utilities.h"

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Point, Point>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

void TestQuaterniondandEuler() {
  Eigen::Quaterniond Qwb;
  Qwb.setIdentity();
  Eigen::Vector3d omega(0, 0, M_PI/10);
  double dt_tmp = 0.005;
  for (double i = 0; i < 20.; i += dt_tmp) {
    Eigen::Quaterniond dq;
    Eigen::Vector3d dtheta_half = omega * dt_tmp / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    Qwb = Qwb * dq;
  }
  std::cout << Qwb.coeffs().transpose() << "\n" << Qwb.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
}

void CreatePointsLines(std::string filename, Points &points, Lines &lines) {
  std::ifstream read_data;
  read_data.open(filename.c_str());
  if (!read_data.is_open()) {
    std::cerr << "Fail to open file: " << filename << std::endl;
    return;
  }

  std::string data_line;
  while (std::getline(read_data, data_line) && !data_line.empty()) {
    std::stringstream ss(data_line);
    double x0, y0, z0;
    double x1, y1, z1;
    ss >> x0 >> y0 >> z0 >> x1 >> y1 >> z1;
    Eigen::Vector4d pt0(x0, y0, z0, 1);
    Eigen::Vector4d pt1(x1, y1, z1, 1);

    bool isHistoryPoint = false;
    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector4d pt = points[i];
      if (pt == pt0) {
        isHistoryPoint = true;
        break;
      }
    }
    if (!isHistoryPoint) {
      points.push_back(pt0);
    }

    isHistoryPoint = false;
    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector4d pt = points[i];
      if (pt == pt1) {
        isHistoryPoint = true;
        break;
      }
    }
    if (!isHistoryPoint) {
      points.push_back(pt1);
    }
    
    lines.emplace_back(pt0, pt1);  // lines
  }

  /// create more 3d points, you can comment this code
  int n = points.size();
  for (int j = 0; j < n; ++j) {
    Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5, 0.5, -0.5, 0);
    points.push_back(p);
  }
  // std::cout << "points.size: " << points.size()
  //           << "\nlines.size: " << lines.size() << std::endl;  // 36 23
  /// save points
  SavePoints("all_points.txt", points);
}

void GenerateAndSaveImuDate(IMU &imuGen, Param &params) {
  std::vector<MotionData> imudata;
  std::vector<MotionData> imudata_noise;
  for (float t = params.t_start; t < params.t_end; ) {
    MotionData data = imuGen.MotionModel(t);
    imudata.push_back(data);

    /// add imu noise
    MotionData data_noise = data;
    imuGen.addIMUnoise(data_noise);
    imudata_noise.push_back(data_noise);

    t += 1.0/params.imu_frequency;
  }
  imuGen.init_velocity_ = imudata[0].imu_velocity;
  imuGen.init_twb_ = imudata[0].twb;
  imuGen.init_Rwb_ = imudata[0].Rwb;

  SaveDataImu("imu_pose.txt", imudata);
  SaveDataImu("imu_pose_noise.txt", imudata_noise);

  SaveGroundtruthAsTUM("groundtruth_tum.txt", imudata);  // save groundtruth for vio test
  SaveImuOutput("imu_output.txt", imudata_noise);  // save imu output for vio test
  SaveEulerAngle("imu_euler_gt.txt", imuGen.euler_angles_all_);
  SaveImuBias("acc_bias_gt.txt", imuGen.acc_bias_all_);
  SaveImuBias("gyro_bias_gt.txt", imuGen.gyro_bias_all_);

  /// test the imu data, integrate the imu data to generate the imu trajecotry
  imuGen.TestImu("imu_pose.txt", "imu_int_pose.txt");
  imuGen.TestImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");
}

void GenerateAndSaveCameraDate(IMU &imuGen, Param &params, Points &points) {
  std::vector<MotionData> camdata;
  for (float t = params.t_start; t < params.t_end; ) {
    MotionData imu = imuGen.MotionModel(t);  // imu body frame to world frame motion
    MotionData cam;

    cam.timestamp = imu.timestamp;
    cam.Rwb = imu.Rwb * params.R_bc;  // Rwc: camera frame to world frame
    cam.twb = imu.twb + imu.Rwb * params.t_bc;  // twc = twb + Rwb*tbc

    camdata.push_back(cam);
    t += 1.0/params.cam_frequency;
  }

  SaveDataCamera("cam_pose.txt", camdata);
  SaveDataCameraAsTUM("camera_pose_tum.txt", camdata);  // for vio test

  /// points observe in image
  for (int n = 0; n < camdata.size(); ++n) {
    MotionData data = camdata[n];
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Twc.block(0, 0, 3, 3) = data.Rwb;
    Twc.block(0, 3, 3, 1) = data.twb;

    /// 遍历所有的特征点，看哪些特征点在视野里
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // 当前camera视野里的特征点的世界坐标
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // camera视野里３维点在当前相机归一化平面上对应点的相机坐标
    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector4d pw = points[i];
      pw[3] = 1;  // 改成齐次坐标

      Eigen::Vector4d pc1 = Twc.inverse() * pw;  // pc = Twc.inverse()*pw  -- > point in cam frame
      if (pc1(2) < 0) continue;  // z必须大于０,在摄像机坐标系前方

      Eigen::Vector2d obs(pc1(0)/pc1(2), pc1(1)/pc1(2));
      // if ((obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
      //     (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w)
      {
        points_cam.push_back(points[i]);
        features_cam.push_back(obs);
      }
    }

    /// save points
    std::stringstream filename;
    filename << "keyframe/landmarks_" << n << ".txt";  // for vio test
    SaveFeatures(filename.str(), points_cam, features_cam);
  }
}

int main() {
  // TestQuaterniondandEuler();

  /// 生成IMU数据并保存
  Param params;
  IMU imuGen(params);
  GenerateAndSaveImuDate(imuGen, params);
  
  /// 生成3d points
  std::string file = "../landmarks_data/house_model/house.txt";
  Points points;
  Lines lines;
  CreatePointsLines(file, points, lines);

  /// 生成Camera数据并保存
  mkdir("keyframe", 0777);
  GenerateAndSaveCameraDate(imuGen, params, points);

  return 0;
}
