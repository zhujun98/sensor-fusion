#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "../../Eigen/Dense"


class KalmanFilter {

public:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd p_;

  // state transition matrix
  Eigen::MatrixXd f_;

  // process covariance matrix
  Eigen::MatrixXd q_;

  // Linear measurement matrix
  Eigen::MatrixXd h_;

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  //
  // A full cycle (prediction + measurement update)
  // using standard Kalman Filter equations
  //
  void KF(const Eigen::VectorXd &z, const Eigen::MatrixXd &r);

  //
  // A full cycle (prediction + measurement update)
  // using extended Kalman Filter equations
  //
  void EKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &r);

private:
  //
  // Prediction Predicts the state and the state covariance
  // using the process model
  //
  void Predict();
};

#endif /* KALMAN_FILTER_H_ */
