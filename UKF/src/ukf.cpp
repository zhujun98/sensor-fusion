#include "ukf.h"
#include "utilities.h"
#include "../../Eigen/Dense"
#include <iostream>


UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_lidar_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  is_initialized_ = false;

  // initial state vector
  n_x_ = 5;

  x_ = Eigen::VectorXd(n_x_);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  P_(0, 0) = 1;
  P_(1, 1) = 1;
  P_(2, 2) = 1;
  P_(3, 3) = 1;
  P_(4, 4) = 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_lidpx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_lidpy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  n_aug_ = 7;

  //
  // Note that the column is augmented but the row is not.
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_+1);

  lambda_ = 3 - n_x_;

  //set weights
  weights_ = Eigen::VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i=1; i<2*n_aug_ + 1; ++i) {
    weights_(i) = 0.5/(lambda_ + n_aug_);
  }

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(const MeasurementPackage &ms_pack) {
  // Initialization
  if (! is_initialized_) {

    if (ms_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = ms_pack.raw_measurements_[0];
      double phi = ms_pack.raw_measurements_[1];
      double v_rho = ms_pack.raw_measurements_[2];

      // Convert positions and velocities from polar to CTRV coordinates.
      double p_x = rho * std::cos(phi);
      double p_y = rho * std::sin(phi);
      x_ << p_x, p_y, v_rho, phi, 0.0;
    } else if (ms_pack.sensor_type_ == MeasurementPackage::LIDAR) {
      double p_x = ms_pack.raw_measurements_[0];
      double p_y = ms_pack.raw_measurements_[1];
      double phi = std::atan2(p_y, p_x);
      x_ << p_x, p_y, 0.0, phi, 0.0;
    } else {
      std::cerr << "Unknown sensor_type_: " << ms_pack.sensor_type_
                << std::endl;
      exit(EXIT_FAILURE);
    }

    time_us_ = ms_pack.timestamp_;

    GenerateSigmaPoints();

    is_initialized_ = true;

    return;
  }

  double dt = (ms_pack.timestamp_ - time_us_)/1000000;

  // Measurement update
  if (ms_pack.sensor_type_ == MeasurementPackage::RADAR
      && use_radar_) {
    UpdateRadar(ms_pack, dt);
  } else if (ms_pack.sensor_type_ == MeasurementPackage::LIDAR
             && use_lidar_) {
    UpdateLidar(ms_pack, dt);
  }

  // The latest time update should be put here since the update may
  // be skipped, e.g. for the RADAR measurement.
  time_us_ = ms_pack.timestamp_;
}

void UKF::Prediction(double delta_t) {
  //
  //Augment the sigma points
  //
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2*n_aug_ + 1);
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);

  //Set augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0.0;
  x_aug(6) = 0.0;

  //Set augmented covariance matrix
  P_aug.block<5, 5>(0, 0) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  //Calculate square root of Psig_aug
  Eigen::MatrixXd Psig_aug = P_aug.llt().matrixL();
  //Set augmented sigma points matrix
  Xsig_aug.col(0) = x_aug;
  for (int i=0; i < n_aug_; ++i) {
    Xsig_aug.col(i+1) = x_aug + std::sqrt(lambda_ + n_aug_)*Psig_aug.col(i);
    Xsig_aug.col(i+n_aug_+1) = x_aug - std::sqrt(lambda_ + n_aug_)*Psig_aug.col(i);
  }
  //
  //Predict sigma points
  //
  for (int i = 0; i< 2*n_aug_ + 1; i++)
  {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //Predicted state values
    double p_x_p, p_y_p;

    //Use different formulas for yawd != 0 and yawd == 0
    if (fabs(yawd) > 0.001) {
      p_x_p = p_x + v/yawd*( sin (yaw + yawd*delta_t) - sin(yaw));
      p_y_p = p_y + v/yawd*( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      p_x_p = p_x + v*delta_t*cos(yaw);
      p_y_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //Add noise
    p_x_p += 0.5*nu_a*delta_t*delta_t * cos(yaw);
    p_y_p += 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p += nu_a*delta_t;
    yaw_p += 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p += nu_yawdd*delta_t;

    //Write predicted sigma point into right column
    Xsig_pred_(0,i) = p_x_p;
    Xsig_pred_(1,i) = p_y_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //
  //predict state mean
  //
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_ + 1; ++i) {
    x_ += weights_(i)*Xsig_pred_.col(i);
  }

  //
  //predict state covariance matrix
  //
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_ + 1; ++i) {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Utilities utilities;

    x_diff(3) = utilities.normalize_angle(x_diff(3));
    P_ += weights_(i)*x_diff*x_diff.transpose();
  }
}

void UKF::UpdateLidar(const MeasurementPackage ms_pack, double delta_t) {

  Prediction(delta_t);

  int n_z = 2;
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2*n_aug_+1);

  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);

  //transform sigma points into measurement space
  for (int i=0; i < 2*n_aug_ + 1; ++i) {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i< 2*n_aug_ + 1; ++i) {
    z_pred += weights_(i)*Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i=0; i< 2*n_aug_ + 1; ++i) {
    Eigen::VectorXd diff = Zsig.col(i) - z_pred;
    S += weights_(i)*diff*diff.transpose();
  }

  //add measurement noise covariance matrix
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << std_lidpx_*std_lidpx_, 0,
      0, std_lidpy_*std_lidpy_;

  S += R;
}


void UKF::UpdateRadar(const MeasurementPackage ms_pack, double delta_t) {

  Prediction(delta_t);

  int n_z = 3;
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2*n_aug_+1);

  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);


  //transform sigma points into measurement space
  for (int i=0; i < 2*n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x*cos(yaw)*v + p_y*sin(yaw)*v)/Zsig(0, i);
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i< 2*n_aug_ + 1; ++i) {
    z_pred += weights_(i)*Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i=0; i< 2*n_aug_ + 1; ++i) {
    Eigen::VectorXd diff = Zsig.col(i) - z_pred;
    S += weights_(i)*diff*diff.transpose();
  }

  //add measurement noise covariance matrix
  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0,std_radrd_*std_radrd_;

  S += R;
}


void UKF::GenerateSigmaPoints() {

  double lambda = 3 - n_x_;

  //calculate square root of P_
  Eigen::MatrixXd A = P_.llt().matrixL();

  Xsig_pred_.col(0) = x_;

  for (int i = 0; i < n_x_; ++i) {
    Xsig_pred_.col(i + 1) = x_ + sqrt(lambda + n_x_)*A.col(i);
    Xsig_pred_.col(i + n_x_ + 1) = x_ - sqrt(lambda + n_x_)*A.col(i);
  }

}