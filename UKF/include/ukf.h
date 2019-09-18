/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef UKF_H
#define UKF_H

#include <Eigen/Dense>

#include "measurement_package.h"


class UKF {

  // initially set to false, set to true in first call of processMeasurement
  bool is_initialized_;

  // time when the state is true, in us
  long long time_us_;

  // if this is false, lidar measurements will be ignored (except for init)
  bool use_lidar_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Lidar measurement noise standard deviation position1 in m
  double std_lidpx_;

  // Lidar measurement noise standard deviation position2 in m
  double std_lidpy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  // Measurement covariance matrices for LIDAR and RADAR.
  Eigen::MatrixXd r_lidar_;
  Eigen::MatrixXd r_radar_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  //
  // prediction Predicts sigma points, the state, and the state covariance matrix.
  // @param delta_t: Time step in s
  //
  void prediction(double delta_t);

  //
  // Updates the state and the state covariance matrix using a lidar measurement.
  // @param m_pkg: Measurement
  // @param delta_t: Time step in second
  //
  void updateLidar(const MeasurementPackage &ms_pack, double delta_t);

  //
  // Updates the state and the state covariance matrix using a radar measurement.
  // @param m_pkg: Measurement
  // @param delta_t: Time step in second
  //
  void updateRadar(const MeasurementPackage &ms_pack, double delta_t);

  //
  // Generate sigma points.
  // @param x: State vector
  // @param P: Covariance matrix
  // @param n_aug: Length of the augmented state vector
  // @param lambda: Sigma point spreading parameter
  //
  Eigen::MatrixXd generateSigmaPoints(const Eigen::VectorXd &x,
                                      const Eigen::MatrixXd &P,
                                      int n_aug,
                                      double lambda);

  //
  // Kalman filter measurement update.
  // @param m_pkg: Measurement
  // @param Zsig: Measurement sigma points matrix
  // @param n_z: Length of the measurement vector
  void measurementUpdate(const MeasurementPackage &ms_pack,
                         const Eigen::MatrixXd &Zsig,
                         int n_z);

public:

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // the current NIS (normalized innovation squared) for radar
  double nis_radar_;

  // the current NIS for lidar
  double nis_lidar_;

  UKF();

  ~UKF();

  //
  // processMeasurement
  // @param ms_pack The latest measurement data of either radar or laser
  //
  void processMeasurement(const MeasurementPackage &ms_pack);

};

#endif /* UKF_H */
