/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include <Eigen/Dense>


class GroundTruthPackage {
public:
  long long timestamp;

  enum SensorType{
    LIDAR,
    RADAR
  } sensor_type;

  Eigen::VectorXd values;

};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
