/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include <iostream>

#include "fusion_ekf.h"
#include "utilities.h"


FusionEKF::FusionEKF() {

  is_initialized_ = false;
  time_us_ = 0;

  use_radar_ = true;
  use_lidar_ = true;

  //state covariance matrix
  ekf_.p_ = Eigen::MatrixXd(4, 4);
  ekf_.p_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //the initial transition matrix F_
  ekf_.f_ = Eigen::MatrixXd(4, 4);
  ekf_.f_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //measurement matrix for the basic Kalman fiter
  ekf_.h_ = Eigen::MatrixXd(2, 4);
  ekf_.h_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  //measurement covariance matrices
  r_lidar_ = Eigen::MatrixXd(2, 2);
  r_lidar_ << 0.0225,      0,
                   0, 0.0225;

  r_radar_ = Eigen::MatrixXd(3, 3);
  r_radar_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;

  //set the acceleration noise components
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;

}

FusionEKF::~FusionEKF() = default;

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack) {

  // Initialization
  if (!is_initialized_) {

    ekf_.x_ = Eigen::VectorXd(4);

    if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
      Eigen::VectorXd x_polar(3);
      x_polar << measurement_pack.values[0],
                 measurement_pack.values[1],
                 measurement_pack.values[2];

      // Convert positions and velocities from polar to cartesian coordinates.
      ekf_.x_ = utilities::polar2Cartesian(x_polar);
    } else if (measurement_pack.sensor_type == MeasurementPackage::LIDAR) {
      ekf_.x_ << measurement_pack.values[0],
                 measurement_pack.values[1],
                                                     0,
                                                     0;
    } else {
      std::cerr << "Unknown sensor_type: " << measurement_pack.sensor_type << std::endl;
      exit(EXIT_FAILURE);
    }

    time_us_ = measurement_pack.timestamp;
    is_initialized_ = true;
    return;
  }

  // Compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp - time_us_) / 1000000.0;  // In second

  // Update the state transition matrix f according to the new elapsed time.
  ekf_.f_(0, 2) = dt;
  ekf_.f_(1, 3) = dt;

  // Update the process noise covariance matrix.
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  ekf_.q_ = Eigen::MatrixXd(4, 4);

  ekf_.q_ << 0.25 * dt_4 * noise_ax_,                       0, 0.5 * dt_3 * noise_ax_,                      0,
                                   0, 0.25 * dt_4 * noise_ay_,                      0, 0.5 * dt_3 * noise_ay_,
              0.5 * dt_3 * noise_ax_,                       0,       dt_2 * noise_ax_,                      0,
                                   0,  0.5 * dt_3 * noise_ay_,                      0,       dt_2 * noise_ay_;

  // Prediction and Measurement updating
  if (measurement_pack.sensor_type == MeasurementPackage::RADAR && use_radar_) {
    ekf_.updateEKF(measurement_pack.values, r_radar_);
  } else if (measurement_pack.sensor_type == MeasurementPackage::LIDAR && use_lidar_) {
    ekf_.update(measurement_pack.values, r_lidar_);
  } else return;

  // Update time stamp
  time_us_ = measurement_pack.timestamp;

}