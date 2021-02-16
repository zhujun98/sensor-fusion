/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef SFND_UKF_HIGHWAY_UTILITIES_H_
#define SFND_UKF_HIGHWAY_UTILITIES_H_

#include <vector>
#include <Eigen/Dense>


namespace utilities {

const double kPI = std::atan(1.0)*4;

//
// Normalize an angle to (-pi, pi]
//
inline double normalizeAngle(double phi) {

  double phi_norm = std::fmod(phi, 2*kPI);
  if (phi_norm <= -kPI) phi_norm += 2*kPI;
  if (phi_norm > kPI) phi_norm -= 2*kPI;

  return phi_norm;
}

};

#endif /* SFND_UKF_HIGHWAY_UTILITIES_H_ */
