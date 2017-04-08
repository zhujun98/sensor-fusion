#include <iostream>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "utilities.h"


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = f_*x_;
  Eigen::MatrixXd f_t = f_.transpose();
  p_ = f_*p_*f_t + q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &r) {

  Eigen::MatrixXd h_t = h_.transpose();
  Eigen::MatrixXd s = h_*p_*h_t + r;
  Eigen::MatrixXd s_i = s.inverse();
  Eigen::MatrixXd k = p_*h_t*s_i;

  x_ = x_ + k*(z - h_*x_);
  long x_size = x_.size();
  Eigen::MatrixXd i = Eigen::MatrixXd::Identity(x_size, x_size);
  p_ = (i - k*h_)*p_;

}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &r) {

  Utilities utilities;

  Eigen::MatrixXd h_j = utilities.CalculateJacobian(x_);
  Eigen::MatrixXd h_t = h_j.transpose();
  Eigen::MatrixXd s = h_j*p_*h_t + r;
  Eigen::MatrixXd s_i = s.inverse();
  Eigen::MatrixXd k = p_*h_t*s_i;

  Eigen::VectorXd x_polar(4);
  x_polar = utilities.Cartesian2Polar(x_);
  x_ = x_ + k*(z - x_polar);
  long x_size = x_.size();
  Eigen::MatrixXd i = Eigen::MatrixXd::Identity(x_size, x_size);
  p_ = (i - k*h_j)*p_;
}
