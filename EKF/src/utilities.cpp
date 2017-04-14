#include <iostream>
#include "utilities.h"
#include "../../Eigen/Dense"


const double kPI = std::atan(1.0)*4;


Utilities::Utilities() {}

Utilities::~Utilities() {}

Eigen::VectorXd Utilities::CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth) {

  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for (std::size_t i=0; i != estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array()*residual.array();

    rmse += residual;
  }

  rmse /= estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}

Eigen::VectorXd Utilities::Cartesian2Polar(const Eigen::VectorXd &x) {

  Eigen::VectorXd x_polar(3);

  double px = x[0];
  double py = x[1];

  double rho = std::sqrt(px*px + py*py);
  double phi = std::atan2(py, px);

  double v_rho = (px*x[2] + py*x[3])/rho;

  x_polar << rho, phi, v_rho;

  return x_polar;
}

Eigen::VectorXd Utilities::Polar2Cartesian(const Eigen::VectorXd &x) {

  Eigen::VectorXd x_cartesian(4);

  double c = std::cos(x[1]);
  double s = std::sin(x[1]);
  double px = x[0] * c;
  double py = x[0] * s;
  double vx = x[2] * c;
  double vy = x[2] * s;

  x_cartesian << px, py, vx, vy;

  return x_cartesian;
}

Eigen::MatrixXd Utilities::CalculateJacobian(const Eigen::VectorXd &x) {

  double px = x[0];
  double py = x[1];

  double c1 = px*px + py*py;
  double c2 = std::sqrt(c1);
  double c3 = c1*c2;

  Eigen::MatrixXd h_j(3, 4);

  h_j << px/c2, py/c2, 0, 0,
      -py/c1, px/c1, 0, 0,
      py*(py*x[2] - px*x[3])/c3, px*(px*x[3] - py*x[2])/c3, px/c2, py/c2;

  return h_j;
}

double Utilities::normalize_angle(double phi) {
  while (phi <= -1.0*kPI) { phi += 2.0*kPI; }
  while (phi > kPI) { phi -= 2.0*kPI; }

  return phi;
}