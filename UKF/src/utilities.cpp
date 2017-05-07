#include <iostream>
#include "utilities.h"


const double kPI = std::atan(1.0)*4;

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

  double phi_norm = std::fmod(phi, 2*kPI);
  if (phi_norm <= -kPI) { phi_norm += 2*kPI; }
  if (phi_norm > kPI) { phi_norm -= 2*kPI; }

  return phi_norm;
}