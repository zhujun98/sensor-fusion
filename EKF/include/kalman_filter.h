/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Eigen/Dense>

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

private:
  //
  // Predict the state and the state covariance using the process model
  //
  void predict();

  //
  // Update the Kalman gain k. Use in both KF() and EKF()
  // @param: p: priori error covariance matrix
  // @param: h: measurement matrix (or the Jacobian)
  // @param: r: measurement covariance matrix
  //
  Eigen::MatrixXd updateKalmanGain(
      const Eigen::MatrixXd &p, const Eigen::MatrixXd &h,
      const Eigen::MatrixXd &r);

public:

  KalmanFilter() = default;

  ~KalmanFilter() = default;

  //
  // A full cycle (prediction + measurement update) using
  // standard Kalman Filter equations
  // @param z: measurement
  // @param r: measurement covariance matrix
  //
  void update(const Eigen::VectorXd &z, const Eigen::MatrixXd &r);

  //
  // A full cycle (prediction + measurement update) using
  // extended Kalman Filter equations
  // @param z: measurement
  // @param r: measurement covariance matrix
  //
  void updateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &r);

};

#endif /* KALMAN_FILTER_H_ */
