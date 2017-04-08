#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "utilities.h"


class Fusion_EKF {
public:

  // Constructor
  Fusion_EKF();

  // Destructor
  virtual ~Fusion_EKF();

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

  // Previous timestamp
  long previous_timestamp_;

  // Acceleration noise components
  float noise_ax_;
  float noise_ay_;

};

#endif /* FusionEKF_H_ */
