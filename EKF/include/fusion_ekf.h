/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <Eigen/Dense>

#include "measurement_package.h"
#include "kalman_filter.h"


class FusionEKF {

  // Check whether the tracking toolbox was initiallized or not
  // (first measurement)
  bool is_initialized_;

  // time when the state is true, in us
  long long time_us_;

  // Acceleration noise components
  double noise_ax_;
  double noise_ay_;

  // if this is false, lidar measurements will be ignored (except for init)
  bool use_lidar_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

public:

  FusionEKF();

  ~FusionEKF();

  KalmanFilter ekf_;

  //
  // Run the whole flow of the Kalman Filter from here.
  //
  void processMeasurement(const MeasurementPackage &measurement_pack);

  //
  // Measurement covariance matrices for LIDAR and RADAR.
  //
  Eigen::MatrixXd r_lidar_;
  Eigen::MatrixXd r_radar_;

};

#endif /* FusionEKF_H_ */
