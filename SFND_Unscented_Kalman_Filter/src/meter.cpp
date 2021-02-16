/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include "meter.hpp"


//
// Calculate root mean square error.
//
Eigen::VectorXd Meter::computeRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                   const std::vector<Eigen::VectorXd> &ground_truth) {

  Eigen::RowVector4d rmse {0, 0, 0, 0};

  for (std::size_t i=0; i != estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();

    rmse += residual;
  }

  rmse /= estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}

void Meter::render(pcl::visualization::PCLVisualizer::Ptr& viewer) {

  viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
//  Eigen::VectorXd rmse = computeRMSE(tools.estimations, tools.ground_truth);
//  viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
//  viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
//  viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
//  viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

//  if(timestamp > 1.0e6)
//  {
//    if(rmse[0] > rmseThreshold[0])
//    {
//      rmseFailLog[0] = rmse[0];
//      pass = false;
//    }
//    if(rmse[1] > rmseThreshold[1])
//    {
//      rmseFailLog[1] = rmse[1];
//      pass = false;
//    }
//    if(rmse[2] > rmseThreshold[2])
//    {
//      rmseFailLog[2] = rmse[2];
//      pass = false;
//    }
//    if(rmse[3] > rmseThreshold[3])
//    {
//      rmseFailLog[3] = rmse[3];
//      pass = false;
//    }
//  }
//  if(!pass)
//  {
//    viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
//    if(rmseFailLog[0] > 0)
//      viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
//    if(rmseFailLog[1] > 0)
//      viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
//    if(rmseFailLog[2] > 0)
//      viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
//    if(rmseFailLog[3] > 0)
//      viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
//  }
}
