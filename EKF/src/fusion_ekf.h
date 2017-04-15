#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "../../Eigen/Dense"
#include "kalman_filter.h"


class FusionEKF {
public:

  // Constructor
  FusionEKF();

  // Destructor
  virtual ~FusionEKF();

  // KalmanFilter() object.
  KalmanFilter ekf_;

  //
  // Run the whole flow of the Kalman Filter from here.
  //
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  //
  // Measurement covariance matrices for LIDAR and RADAR.
  //
  Eigen::MatrixXd r_lidar_;
  Eigen::MatrixXd r_radar_;

private:

  // Check whether the tracking toolbox was initiallized or not
  // (first measurement)
  bool is_initialized_;

  // time when the state is true, in us
  long long time_us_;

  // Acceleration noise components
  double noise_ax_;
  double noise_ay_;

};

#endif /* FusionEKF_H_ */
