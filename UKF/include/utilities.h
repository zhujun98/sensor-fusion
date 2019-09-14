#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <Eigen/Dense>

#include <vector>


namespace Utilities {
  //
  // Calculate root mean square error.
  //
  Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd> &estimations,
      const std::vector<Eigen::VectorXd> &ground_truth);

  //
  // Normalize an angle to (-pi, pi]
  //
  double normalize_angle(double phi);
};

#endif /* UTILITIES_H_ */
