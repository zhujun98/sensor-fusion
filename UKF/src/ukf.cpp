#include "ukf.h"
#include "utilities.h"
#include <iostream>


UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_lidar_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  is_initialized_ = false;

  // Length of initial state vector
  n_x_ = 5;

  x_ = Eigen::VectorXd(n_x_);

  // Initial covariance matrix
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

  //measurement covariance matrices for LIDAR and RADIA
  r_lidar_ = Eigen::MatrixXd(2, 2);
  r_lidar_ << std_lidpx_*std_lidpx_, 0,
              0, std_lidpy_*std_lidpy_;

  r_radar_ = Eigen::MatrixXd(3, 3);
  r_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;

  // Length of augmented state vector
  n_aug_ = 7;

  // Note that the column is augmented but the row is not.
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_+1);

  // Scaling parameter for unscented transformation.
  lambda_ = 3 - n_aug_;

  // Set weights
  weights_ = Eigen::VectorXd(2*n_aug_ + 1);
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
      // double v_rho = ms_pack.raw_measurements_[2];

      // Convert positions and velocities from polar to CTRV coordinates.
      double p_x = rho * std::cos(phi);
      double p_y = rho * std::sin(phi);
      x_ << p_x, p_y, 0.0, phi, 0.0;

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

    Xsig_pred_ = GenerateSigmaPoints(x_, P_, n_aug_, lambda_);

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
  Xsig_aug = GenerateSigmaPoints(x_aug, P_aug, n_aug_, lambda_);

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
    if (std::abs(yawd) > 0.001) {
      p_x_p = p_x + v/yawd*(std::sin(yaw + yawd*delta_t) - std::sin(yaw));
      p_y_p = p_y + v/yawd*(std::cos(yaw) - std::cos(yaw + yawd*delta_t));
    }
    else {
      p_x_p = p_x + v*delta_t*cos(yaw);
      p_y_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //Add noise
    double delta_t2 = delta_t*delta_t;
    p_x_p += 0.5*nu_a*delta_t2*std::cos(yaw);
    p_y_p += 0.5*nu_a*delta_t2*std::sin(yaw);
    v_p += nu_a*delta_t;
    yaw_p += 0.5*nu_yawdd*delta_t2;
    yawd_p += nu_yawdd*delta_t;

    //Write predicted sigma point into right column
    Xsig_pred_(0, i) = p_x_p;
    Xsig_pred_(1, i) = p_y_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
  //
  //Calculate state mean
  //
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i) {
    x_ += weights_(i)*Xsig_pred_.col(i);
  }

  //
  //Calculate state covariance matrix
  //
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i) {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //Utilities utilities;
    //x_diff(3) = utilities.normalize_angle(x_diff(3));

    P_ += weights_(i)*x_diff*x_diff.transpose();
  }

  std::cout << x_ << std::endl;
}

void UKF::UpdateLidar(const MeasurementPackage &ms_pack, double delta_t) {

  Prediction(delta_t);

  Utilities utilities;

  int n_z = 2;
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2*n_aug_+1);

  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; ++i) {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  MeasurementUpdate(ms_pack, Zsig, n_z);
}

void UKF::UpdateRadar(const MeasurementPackage &ms_pack, double delta_t) {

  Prediction(delta_t);

  Utilities utilities;

  int n_z = 3;
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2*n_aug_+1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_+1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double rho = sqrt(p_x * p_x + p_y * p_y);
    if (std::abs(rho) < 1e-6) { rho = 1e-6; }
    Zsig(0, i) = rho;
    Zsig(1, i) = atan2(p_y, p_x);
    // Avoid division by zero
    Zsig(2, i) = (p_x * cos(yaw) * v + p_y * sin(yaw) * v) / rho;
  }

  MeasurementUpdate(ms_pack, Zsig, n_z);
}

void UKF::MeasurementUpdate(
    const MeasurementPackage &ms_pack, const Eigen::MatrixXd &Zsig, int n_z) {

  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);

  Utilities utilities;

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i< 2*n_aug_+1; ++i) {
    z_pred += weights_(i)*Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);

  S.fill(0.0);
  for (int i=0; i< 2*n_aug_+1; ++i) {
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
    //if (ms_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //  z_diff(1) = utilities.normalize_angle(z_diff(1));
    //}
    S += weights_(i)*z_diff*z_diff.transpose();
  }

  if (ms_pack.sensor_type_ == MeasurementPackage::RADAR) {
    S += r_radar_;
  } else if (ms_pack.sensor_type_ == MeasurementPackage::LIDAR){
    S += r_lidar_;
  }

  //calculate cross correlation matrix
  Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    //residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
    //if (ms_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //  z_diff(1) = utilities.normalize_angle(z_diff(1));
    //}

    // state difference
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //x_diff(3) = utilities.normalize_angle(x_diff(3));

    Tc += weights_(i)*x_diff*z_diff.transpose();
  }

  //Kalman gain K;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = Tc*Si;

  //residual
  Eigen::VectorXd z_diff = ms_pack.raw_measurements_ - z_pred;
  //if (ms_pack.sensor_type_ == MeasurementPackage::RADAR) {
  //  z_diff(1) = utilities.normalize_angle(z_diff(1));
  //}
  //update state mean and covariance matrix
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();

  // Calculate the normalized innovation squared()
  if (ms_pack.sensor_type_ == MeasurementPackage::RADAR) {
    NIS_radar_ = z_diff.transpose()*Si*z_diff;
  } else if (ms_pack.sensor_type_ == MeasurementPackage::LIDAR){
    NIS_lidar_ = z_diff.transpose()*Si*z_diff;
  }

}

Eigen::MatrixXd UKF::GenerateSigmaPoints(
    const Eigen::VectorXd &x, const Eigen::MatrixXd &P, int n_aug, double lambda) {

  long n = x.size();

  Eigen::MatrixXd A = P.llt().matrixL();
  Eigen::MatrixXd Xsig = Eigen::MatrixXd(n, 2*n_aug+1);

  Xsig.col(0) = x;
  for (int i = 0; i < n; ++i) {
    Xsig.col(i + 1) = x + sqrt(lambda + n)*A.col(i);
    Xsig.col(i + n + 1) = x - sqrt(lambda + n)*A.col(i);
  }

  return Xsig;
}
