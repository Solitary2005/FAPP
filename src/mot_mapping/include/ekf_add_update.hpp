#pragma once
#include <ros/ros.h>
#include <queue>
#include <deque>
#include <Eigen/Geometry>
// 卡尔曼滤波器，用来对动态物体进行跟踪
struct EkfNew {
  typedef std::shared_ptr<EkfNew> Ptr;
  int id;
  double dt;
  ros::Time last_update_stamp_;
  int age, update_num;
  // 状态转移矩阵A，控制矩阵B，观测矩阵C（因为这里假设传感器和状态的尺度和单位一致所以是单位阵）
  Eigen::MatrixXd A, B, C;
  // R是测量噪声的协方差矩阵，Q是环境噪声(过程噪声)的协方差矩阵
  Eigen::MatrixXd Qt, Rt;
  // Sigma是协方差矩阵，K是卡尔曼增益
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  int window_size = 10;                         // 滑动窗口大小，可调
  std::deque<Eigen::MatrixXd> InnoCov_list;     // 已有：保存 γγ^T
  Eigen::MatrixXd P_post_prev;                  // 缓存 P_{k-1|k-1}

  EkfNew(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 6);
    C.setZero(6, 6);
    // 假设匀加速运动模型 
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 3) = dt;
    B(4, 4) = dt;
    B(5, 5) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    C(3, 3) = 1;
    C(4, 4) = 1;
    C(5, 5) = 1;
    K = C;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt(0, 0) = 0.1;
    Qt(1, 1) = 0.1;
    Qt(2, 2) = 0.1;
    Qt(3, 3) = 0.1;
    Qt(4, 4) = 0.1;
    Qt(5, 5) = 0.1;
    Rt(0, 0) = 0.09;
    Rt(1, 1) = 0.09;
    Rt(2, 2) = 0.09;
    Rt(3, 3) = 0.4; //0.4
    Rt(4, 4) = 0.4;
    Rt(5, 5) = 0.4;
    x.setZero(6);
    P_post_prev.setZero(6, 6);
    InnoCov_list.clear();
  }
  inline void predict() {
    // P_post_prev = Sigma; // 缓存 P_{k-1|k-1}
    x = A * x; // 状态预测
    Sigma = A * Sigma * A.transpose() + Qt; // 更新协方差
    return;
  }
  inline void reset(const Eigen::Vector3d& z, int id_) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
    last_update_stamp_ = ros::Time::now();
    age = 1;
    update_num = 0;
    id = id_;
  }
  inline bool checkValid(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }
  inline void update(const Eigen::Vector3d& z1, const Eigen::Vector3d& z2) {
    Eigen::VectorXd z(6);
    z << z1, z2;
    // // 1) 计算创新 γk（此时 x 仍是预测态）
    // Eigen::VectorXd gamma = z - C * x;

    // // 2) 维护滑动窗口并计算 Cγ,k = 平均(γγ^T)
    // Eigen::MatrixXd gg = gamma * gamma.transpose();
    // InnoCov_list.push_back(gg);
    // if ((int)InnoCov_list.size() > window_size) InnoCov_list.pop_front();

    // Eigen::MatrixXd Cgamma = Eigen::MatrixXd::Zero(6, 6);
    // for (const auto& M : InnoCov_list) Cgamma += M;
    // Cgamma /= static_cast<double>(InnoCov_list.size());

    // // 3) 计算 Q_hat
    // Eigen::MatrixXd APAT = A * P_post_prev * A.transpose();
    // Eigen::MatrixXd Q_hat = Cgamma - APAT - Rt;
    // // Eigen::MatrixXd Q_hat = Cgamma - APAT;

    // // 4) 正定性保证：仅保留非负对角
    // for (int i = 0; i < 6; ++i) {
    //     double v = Q_hat(i, i);
    //     Qt(i, i) = std::max(0.0, v);
    // }
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
    
    last_update_stamp_ = ros::Time::now();
    update_num ++;
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
};