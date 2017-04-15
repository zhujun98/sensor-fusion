#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <vector>
#include "../../Eigen/Dense"


class Utilities {
public:

  // Constructor
  Utilities();

  // Destructor
  virtual ~Utilities();

  //
  // Calculate root mean square error.
  //
  Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd> &estimations,
      const std::vector<Eigen::VectorXd> &ground_truth);

  //
  // Convert Cartesian coordinates to polar coordinates.
  //
  Eigen::VectorXd Cartesian2Polar(const Eigen::VectorXd &x);

  //
  // Convert polar coordinates to Cartesian coordinates.
  // Only used in the initialization of FusionEKF().
  //
  Eigen::VectorXd Polar2Cartesian(const Eigen::VectorXd &x);

  //
  // Calculate Jacobian.
  //
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x);

  //
  // Normalize the angle to [-pi, pi]
  //
  double normalize_angle(double phi);

};

#endif /* UTILITIES_H_ */
