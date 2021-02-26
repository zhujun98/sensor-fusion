/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */
#ifndef SFND_UKF_HIGHWAY_MEASUREMENT_PACKAGE_H_
#define SFND_UKF_HIGHWAY_MEASUREMENT_PACKAGE_H_

#include <Eigen/Dense>


struct MeasurementPackage {
  long long timestamp;

  enum SensorType{
    LIDAR,
    RADAR
  } sensor_type;

  Eigen::VectorXd values;

};

#endif /* SFND_UKF_HIGHWAY_MEASUREMENT_PACKAGE_H_ */