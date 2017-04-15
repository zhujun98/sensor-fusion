#include <iostream>
#include "utilities.h"


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

double Utilities::normalize_angle(double phi) {
  if (phi <= -1.0*kPI || phi > kPI) {
    double s = std::sin(phi);
    phi = std::asin(s);
  }

  return phi;
}