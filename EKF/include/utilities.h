/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <vector>
#include <Eigen/Dense>


namespace utilities {

const double kPI = std::atan(1.0)*4;

//
// Convert Cartesian coordinates to polar coordinates.
//
inline Eigen::VectorXd cartesian2Polar(const Eigen::VectorXd &x) {

  Eigen::VectorXd x_polar(3);

  double px = x[0];
  double py = x[1];

  double rho = std::sqrt(px*px + py*py);
  double phi = std::atan2(py, px);

  if (rho < 1e-6 ) { rho = 1e-6; }

  double v_rho = (px*x[2] + py*x[3])/rho;

  x_polar << rho, phi, v_rho;

  return x_polar;
}

//
// Convert polar coordinates to Cartesian coordinates.
// Only used in the initialization of FusionEKF().
//
inline Eigen::VectorXd polar2Cartesian(const Eigen::VectorXd &x) {

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

//
// Calculate Jacobian.
//
inline Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd &x) {

  double px = x[0];
  double py = x[1];
  double vx = x[2];
  double vy = x[3];

  double c1 = px*px + py*py;
  if (c1 < 1e-12 ) { c1 = 1e-12; }
  double c2 = std::sqrt(c1);
  double c3 = c1*c2;

  Eigen::MatrixXd h_j(3, 4);

  h_j << px/c2, py/c2, 0, 0,
      -py/c1, px/c1, 0, 0,
      py*(py*vx - px*vy)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return h_j;
}

//
// Normalize the angle to [-pi, pi]
//
inline double normalizeAngle(double phi) {
  if (phi < -1.0*kPI || phi > kPI) {
    double s = std::sin(phi);
    phi = std::asin(s);
  }

  return phi;
}

} // utilities

#endif /* UTILITIES_H_ */
