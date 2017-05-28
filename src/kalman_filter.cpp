#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = h(x_);
  VectorXd y = z - z_pred;
  const double PI = 3.14159265358979323846;
  
  while (y(1) < -PI)
    y(1) += 2 * PI;
  while (y(1) > PI)
    y(1) -= 2 * PI;
  
  MatrixXd Hjt = H_.transpose();
  MatrixXd S = H_ * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}


VectorXd KalmanFilter::h(const VectorXd &x) {
  VectorXd z_pred = VectorXd(3);
  
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);
  z_pred(0) = sqrt(px*px + py*py);
  
  if (fabs(px) > 0.0001) {
    z_pred(1) = atan2(py, px);
    
    if (fabs(z_pred(0)) > 0.0001) {
      z_pred(2) = (px*vx + py*vy) / z_pred(0);
    } else {
      z_pred(2) = 0.0;
    }
  } else {
    z_pred(1) = 0.0;
    z_pred(2) = 0.0;
  }
  return z_pred;
}
