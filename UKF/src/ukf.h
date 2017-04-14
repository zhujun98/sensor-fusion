#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "../../Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "utilities.h"


class UKF {

public:

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // the current NIS for radar
  double NIS_radar_;

  // the current NIS for laser
  double NIS_laser_;

  // Constructor
  UKF();

  // Destructor
  virtual ~UKF();

  //
  // ProcessMeasurement
  // @param ms_pack The latest measurement data of either radar or laser
  //
  void ProcessMeasurement(const MeasurementPackage &ms_pack);

private:

  // initially set to false, set to true in first call of ProcessMeasurement
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
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  //
  // Prediction Predicts sigma points, the state, and the state covariance matrix
  // @param delta_t Time between k and k+1 in s
  //
  void Prediction(double delta_t);

  //
  // Updates the state and the state covariance matrix using a lidar measurement
  // @param meas_package The measurement at k+1
  //
  void UpdateLidar(const MeasurementPackage ms_pack, double delta_t);

  //
  // Updates the state and the state covariance matrix using a radar measurement
  // @param meas_package The measurement at k+1
  //
  void UpdateRadar(const MeasurementPackage ms_pack, double delta_t);

  //
  // Generate sigma points.
  //
  void GenerateSigmaPoints();

};

#endif /* UKF_H */
