#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "utilities.h"


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
  // Measurement covariance matrices and measurement matrices for
  // LIDAR and RADAR.
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
  float noise_ax_;
  float noise_ay_;

};

#endif /* FusionEKF_H_ */
