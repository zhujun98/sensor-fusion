#include <iostream>
#include "kalman_filter.h"
#include "utilities.h"


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = f_*x_;
  Eigen::MatrixXd f_t = f_.transpose();
  p_ = f_*p_*f_t + q_;
}

void KalmanFilter::KF(const Eigen::VectorXd &z, const Eigen::MatrixXd &r) {

  Predict();

  // Measurement update
  Eigen::MatrixXd k;
  k = UpdateKalmanGain(p_, h_, r);

  x_ = x_ + k*(z - h_*x_);
  long x_size = x_.size();
  Eigen::MatrixXd i = Eigen::MatrixXd::Identity(x_size, x_size);
  p_ = (i - k*h_)*p_;
}

void KalmanFilter::EKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &r) {

  Predict();

  // Measurement update
  Utilities utilities;

  Eigen::MatrixXd h_j = utilities.CalculateJacobian(x_);

  Eigen::MatrixXd k;
  k = UpdateKalmanGain(p_, h_j, r);

  Eigen::VectorXd x_polar(4);
  x_polar = utilities.Cartesian2Polar(x_);
  x_ = x_ + k*(z - x_polar);
  long x_size = x_.size();
  Eigen::MatrixXd i = Eigen::MatrixXd::Identity(x_size, x_size);
  p_ = (i - k*h_j)*p_;
}

Eigen::MatrixXd KalmanFilter::UpdateKalmanGain(
    const Eigen::MatrixXd &p, const Eigen::MatrixXd &h,
    const Eigen::MatrixXd &r) {
  Eigen::MatrixXd h_t = h.transpose();
  Eigen::MatrixXd ph_t = p*h_t;
  Eigen::MatrixXd s = h*ph_t + r;
  Eigen::MatrixXd s_i = s.inverse();
  Eigen::MatrixXd k = ph_t*s_i;

  return k;
}