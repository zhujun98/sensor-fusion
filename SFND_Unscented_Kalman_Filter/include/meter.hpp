/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef SFND_UKF_HIGHWAY_METER_H
#define SFND_UKF_HIGHWAY_METER_H

#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>


class Meter {

public:

  Eigen::VectorXd computeRMSE(const std::vector<Eigen::VectorXd>& estimations,
                              const std::vector<Eigen::VectorXd>& ground_truth);

  void render(pcl::visualization::PCLVisualizer::Ptr& viewer);
};


#endif //SFND_UKF_HIGHWAY_METER_H
