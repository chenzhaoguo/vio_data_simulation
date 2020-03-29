#include <random>
#include "imu.h"
#include "utilities.h"

Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles) {  // ZYX
  double roll = eulerAngles(0);
  double pitch = eulerAngles(1);
  double yaw = eulerAngles(2);
  double cr = cos(roll); double sr = sin(roll);
  double cp = cos(pitch); double sp = sin(pitch);
  double cy = cos(yaw); double sy = sin(yaw);
  Eigen::Matrix3d R;
  R << cy*cp,  cy*sp*sr - sy*cr,  sy*sr + cy*cr*sp,
       sy*cp,  cy*cr + sy*sr*sp,  sp*sy*cr - cy*sr,
       -sp,    cp*sr,             cp*cr;
  return R;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles) {
  double roll = eulerAngles(0);
  double pitch = eulerAngles(1);
  double cr = cos(roll); double sr = sin(roll);
  double cp = cos(pitch); double sp = sin(pitch);
  Eigen::Matrix3d Ebw;
  Ebw << 1,  0,    -sp,
         0,  cr,   sr*cp,
         0,  -sr,  cr*cp;
  return Ebw;
}

IMU::IMU(Param p) : param_(p) {
  gyro_bias_ = Eigen::Vector3d::Zero();
  acc_bias_ = Eigen::Vector3d::Zero();
}

MotionData IMU::MotionModel(double t) {
  /// translation
  Eigen::Vector3d position;
  Eigen::Vector3d dp;  // v
  Eigen::Vector3d ddp;  // a
  if (t >= 0 && t <= 10) {
    position = Eigen::Vector3d(13, 0.12*t*t, 0);
    dp = Eigen::Vector3d(0, 0.24*t, 0);
    ddp = Eigen::Vector3d(0, 0.24, 0);
  } else if (t > 10 && t <= 13) {
    position = Eigen::Vector3d(12+cos((t-10)*M_PI/6), 12+sin((t-10)*M_PI/6), 0);
    dp = Eigen::Vector3d(-M_PI/6*sin((t-10)*M_PI/6), M_PI/6*cos((t-10)*M_PI/6), 0);
    ddp = Eigen::Vector3d(-M_PI/6*M_PI/6*cos((t-10)*M_PI/6), -M_PI/6*M_PI/6*sin((t-10)*M_PI/6), 0);
  } else if (t > 13 && t <= 21) {
    position = Eigen::Vector3d(12-2.4*(t-13)+0.1*(t-13)*(t-13), 13, 0);
    dp = Eigen::Vector3d(-2.4+0.2*(t-13), 0, 0);
    ddp = Eigen::Vector3d(0.2, 0, 0);
  } else if (t > 21 && t <= 35) {
    position = Eigen::Vector3d(-0.8-0.8*(t-21), 13, 0);
    dp = Eigen::Vector3d(-0.8, 0, 0);
    ddp = Eigen::Vector3d(0, 0, 0);
  } else if (t > 35 && t <= 37) {
    position = Eigen::Vector3d(-12-sin((t-35)*M_PI/4), 12+cos((t-35)*M_PI/4), 0);
    dp = Eigen::Vector3d(-M_PI/4*cos((t-35)*M_PI/4), -M_PI/4*sin((t-35)*M_PI/4), 0);
    ddp = Eigen::Vector3d(M_PI/4*M_PI/4*sin((t-35)*M_PI/4), -M_PI/4*M_PI/4*cos((t-35)*M_PI/4), 0);
  } else if (t > 37 && t <= 42) {
    position = Eigen::Vector3d(-13, 12-0.8*(t-37), 0);
    dp = Eigen::Vector3d(0, -0.8, 0);
    ddp = Eigen::Vector3d(0, 0, 0);
  } else if (t > 42 && t <= 54) {
    position = Eigen::Vector3d(-13, -0.8*(t-42)-0.5*0.05*(t-42)*(t-42), 0);
    dp = Eigen::Vector3d(0, -0.8-0.05*(t-42), 0);
    ddp = Eigen::Vector3d(0, -0.05, 0);
  } else if (t > 54 && t <= 57) {
    position = Eigen::Vector3d(-12-cos((t-54)*M_PI/6), -13.2-sin((t-54)*M_PI/6), 0);
    dp = Eigen::Vector3d(M_PI/6*sin((t-54)*M_PI/6), -M_PI/6*cos((t-54)*M_PI/6), 0);
    ddp = Eigen::Vector3d(M_PI/6*M_PI/6*cos((t-54)*M_PI/6), M_PI/6*M_PI/6*sin((t-54)*M_PI/6), 0);
  } else if (t > 57 && t <= 67) {
    position = Eigen::Vector3d(-12+1.4*(t-57)+0.03*(t-57)*(t-57), -14.2, 0);
    dp = Eigen::Vector3d(1.4+0.06*(t-57), 0, 0);
    ddp = Eigen::Vector3d(0.06, 0, 0);
  } else if (t > 67 && t <= 72) {
    position = Eigen::Vector3d(5+2*(t-67)-0.12*(t-67)*(t-67), -14.2, 0);
    dp = Eigen::Vector3d(2-0.24*(t-67), 0, 0);
    ddp = Eigen::Vector3d(-0.24, 0, 0);
  } else if (t > 72 && t <= 75) {
    position = Eigen::Vector3d(12+sin((t-72)*M_PI/6), -13.2-cos((t-72)*M_PI/6), 0);
    dp = Eigen::Vector3d(M_PI/6*cos((t-72)*M_PI/6), M_PI/6*sin((t-72)*M_PI/6), 0);
    ddp = Eigen::Vector3d(-M_PI/6*M_PI/6*sin((t-72)*M_PI/6), M_PI/6*M_PI/6*cos((t-72)*M_PI/6), 0);
  } else if (t > 75 && t <= 91) {
    position = Eigen::Vector3d(13, -13.2+0.825*(t-75), 0);
    dp = Eigen::Vector3d(0, 0.825, 0);
    ddp = Eigen::Vector3d(0, 0, 0);
  }
  position += Eigen::Vector3d(0, 0, 1.5);

  /// Rotation
  double k_roll = 0.1;  // roll of body frame to world frame
  double k_pitch = 0.2;  // pitch of body frame to world frame
  // Eigen::Vector3d eulerAngles(k_roll*cos(t), k_pitch*sin(t), 0);
  // Eigen::Vector3d eulerAnglesRates(-k_roll*sin(t), k_pitch*cos(t), 0);  // eulerAngles的导数：W系下的欧拉角速度
  Eigen::Vector3d eulerAngles(0, 0, 0);
  Eigen::Vector3d eulerAnglesRates(0, 0, 0);  // eulerAngles的导数：W系下的欧拉角速度
  if (t > 10 && t <= 13) {
    eulerAngles += Eigen::Vector3d(0, 0, M_PI/6*(t-10));
    eulerAnglesRates += Eigen::Vector3d(0, 0, M_PI/6);
  } else if (t > 13 && t <= 35) {
    eulerAngles += Eigen::Vector3d(0, 0, M_PI/2);
  } else if (t > 35 && t <= 37) {
    eulerAngles += Eigen::Vector3d(0, 0, M_PI/2+M_PI/4*(t-35));
    eulerAnglesRates += Eigen::Vector3d(0, 0, M_PI/4);
  } else if (t > 37 && t <= 54) {
    eulerAngles += Eigen::Vector3d(0, 0, M_PI);
  } else if (t > 54 && t <= 57) {
    eulerAngles += Eigen::Vector3d(0, 0, -M_PI+M_PI/6*(t-54));
    eulerAnglesRates += Eigen::Vector3d(0, 0, M_PI/6);
  } else if (t > 57 && t <= 72) {
    eulerAngles += Eigen::Vector3d(0, 0, -M_PI/2);
  } else if (t > 72 && t <= 75) {
    eulerAngles += Eigen::Vector3d(0, 0, -M_PI/2+M_PI/6*(t-72));
    eulerAnglesRates += Eigen::Vector3d(0, 0, M_PI/6);
  }
  euler_angles_all_.insert(std::make_pair(t, eulerAngles));
  /// generate gyro data
  Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;  // euler rates trans to body gyro

  /// generate acc data
  Eigen::Matrix3d Rwb;  // body frame to world frame
  /// 先绕x轴转动roll，再绕y轴转动pitch，最后绕z转动yaw
  Rwb = Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX());
  Eigen::Vector3d gn(0, 0, -9.81);  // gravity in navigation frame(ENU)
  Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - gn);

  MotionData data;
  data.timestamp = t;
  data.Rwb = Rwb;
  data.twb = position;
  data.imu_acc = imu_acc;
  data.imu_gyro = imu_gyro;
  data.imu_velocity = dp;
  return data;
}

void IMU::addIMUnoise(MotionData &data) {
  std::random_device rd;
  std::default_random_engine generator_(rd());
  std::normal_distribution<double> noise(0.0, 1.0);

  Eigen::Vector3d noise_gyro(noise(generator_), noise(generator_), noise(generator_));
  Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
  data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt(param_.imu_timestep) + gyro_bias_;

  Eigen::Vector3d noise_acc(noise(generator_), noise(generator_), noise(generator_));
  Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
  data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt(param_.imu_timestep) + acc_bias_;

  /// save imu bias
  acc_bias_all_.insert(std::make_pair(data.timestamp, acc_bias_));
  gyro_bias_all_.insert(std::make_pair(data.timestamp, gyro_bias_));

  /// gyro_bias update
  Eigen::Vector3d noise_gyro_bias(noise(generator_), noise(generator_), noise(generator_));
  gyro_bias_ = gyro_bias_ + noise_gyro_bias * param_.gyro_bias_sigma * sqrt(param_.imu_timestep);
  data.imu_gyro_bias = gyro_bias_;

  /// acc_bias update
  Eigen::Vector3d noise_acc_bias(noise(generator_), noise(generator_), noise(generator_));
  acc_bias_ = acc_bias_ + noise_acc_bias * param_.acc_bias_sigma * sqrt(param_.imu_timestep);
  data.imu_acc_bias = acc_bias_;
}

/// 读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，用来验证数据以及模型的有效性。
void IMU::TestImu(std::string src, std::string dest) {
  std::vector<MotionData> imu_data;
  LoadPose(src, imu_data);

  std::ofstream save_result;
  save_result.open(dest);

  double dt = param_.imu_timestep;
  Eigen::Vector3d Pwb = imu_data[0].twb;  // position: from imu measurements
  Eigen::Quaterniond Qwb(imu_data[0].Rwb);  // quaterniond: from imu measurements
  Eigen::Vector3d Vw = init_velocity_;  // initial velocity
  Eigen::Vector3d gn(0, 0, -9.81);  // ENU frame

  for (int i = 1; i < imu_data.size(); ++i) {
    MotionData imu_pose_last = imu_data[i-1];  // k时刻
    MotionData imu_pose_curr = imu_data[i];  // k+1时刻

    /// 中值积分
    Eigen::Quaterniond dq;  // delta_q = [1, 0.5*wb*delta_t]
    Eigen::Vector3d dtheta_half = 0.5 * (imu_pose_last.imu_gyro + imu_pose_curr.imu_gyro) * dt / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    Eigen::Quaterniond Qwb_last = Qwb;
    Qwb = Qwb * dq;
    Qwb.normalize();
    Eigen::Vector3d acc_w = 0.5 * (Qwb_last * imu_pose_last.imu_acc + Qwb * imu_pose_curr.imu_acc) + gn;  // acc_world = Rwb * acc_body + gn
    Pwb = Pwb + Vw * dt + 0.5 * acc_w * dt * dt;  // 先更新P
    Vw = Vw + acc_w * dt;  // 再更新V

    /// imu 动力学模型 欧拉积分
    /// 第一种欧拉积分：采用k时刻gyro、acc的值
    /*
    Eigen::Quaterniond dq;  // delta_q = [1, 0.5*wb*delta_t]
    Eigen::Vector3d dtheta_half = imu_pose_last.imu_gyro * dt / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    Eigen::Vector3d acc_w = Qwb * imu_pose_last.imu_acc + gn;  // acc_world = Rwb * acc_body + gn
    Qwb = Qwb * dq;
    Qwb.normalize();
    Pwb = Pwb + Vw * dt + 0.5 * acc_w * dt * dt;
    Vw = Vw + acc_w * dt;
    */
    /// 第二种欧拉积分：采用k+1时刻gyro、acc的值
    /*
    Eigen::Quaterniond dq;  // delta_q = [1, 0.5*wb*delta_t]
    Eigen::Vector3d dtheta_half = imu_pose_curr.imu_gyro * dt / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    Qwb = Qwb * dq;
    Qwb.normalize();
    Eigen::Vector3d acc_w = Qwb * imu_pose_curr.imu_acc + gn;  // acc_world = Rwb * acc_body + gn
    Pwb = Pwb + Vw * dt + 0.5 * acc_w * dt * dt;
    Vw = Vw + acc_w * dt;
    */

    Eigen::Vector3d euler = Quaterniond2EulerAngle(Qwb);  // output: roll/pitch/yaw

    ///　按着imu quaternion, imu postion, cam quaternion, cam postion的格式存储，由于没有cam，所以imu存了两次
    save_result << imu_pose_curr.timestamp << " "
                << Qwb.w() << " "
                << Qwb.x() << " "
                << Qwb.y() << " "
                << Qwb.z() << " "
                << Pwb(0) << " "
                << Pwb(1) << " "
                << Pwb(2) << " "
                << euler[0] << " "
                << euler[1] << " "
                << euler[2] << std::endl;
  }
  std::cout << "imu integration end!" << std::endl;
}
