/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */
#ifndef SFND_UKF_HIGHWAY_CAR_H
#define SFND_UKF_HIGHWAY_CAR_H

#include <pcl/visualization/pcl_visualizer.h>
#include "ukf.hpp"


class Car
{
  std::string name_;

  // units in meters
  Eigen::Vector3f pos_; // (x, y, z)
  Eigen::Vector3f size_; // (w, h, d)
  Eigen::Quaternionf orient_;
  Eigen::Vector3f color_; // (r, g, b)

  Eigen::Vector3f velocity_; // (vx, vy, vz)
  Eigen::Vector3f acc_; // (ax, ay, az)

  float steering;
  // distance between front of vehicle and center of gravity
  float Lf;

  UKF ukf_;

  // accuation instructions
//  std::vector<accuation> instructions;
//  int accuateIndex;

//  double sinNegTheta;
//  double cosNegTheta;

public:

  Car(const std::string& name,
      const Eigen::Vector3f& size = {4.f, 2.f, 2.f},
      const Eigen::Vector3f& color = {1.f, 0.f, 0.f});

  ~Car() = default;

  const Eigen::Vector3f& position() const;

  const std::string& name() const;

  void setColor(const Eigen::Vector3f& color);

  void moveTo(const Eigen::Vector3f& pos);

  void step(float dt, size_t timestamp);

//  // angle around z axis
//  Eigen::Quaternionf getQuaternion(float theta)
//  {
//    Eigen::Matrix3f rotation_mat;
//    rotation_mat <<
//                 cos(theta), -sin(theta), 0,
//    sin(theta),  cos(theta), 0,
//    0, 			 0, 		 1;
//
//    Eigen::Quaternionf q(rotation_mat);
//    return q;
//  }
//
//  void setAcceleration(float setAcc)
//  {
//    acceleration = setAcc;
//  }
//
//  void setSteering(float setSteer)
//  {
//    steering = setSteer;
//  }
//
//  void setInstructions(std::vector<accuation> setIn)
//  {
//    for(accuation a : setIn)
//      instructions.push_back(a);
//  }
//
//  // collision helper function
//  bool inbetween(double point, double center, double range)
//  {
//    return (center - range <= point) && (center + range >= point);
//  }
//
//  bool checkCollision(Vect3 point)
//  {
//    // check collision for rotated car
//    double xPrime = ((point.x-position.x) * cosNegTheta - (point.y-position.y) * sinNegTheta)+position.x;
//    double yPrime = ((point.y-position.y) * cosNegTheta + (point.x-position.x) * sinNegTheta)+position.y;
//
//    return (inbetween(xPrime, position.x, dimensions.x / 2) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
//           (inbetween(xPrime, position.x, dimensions.x / 4) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
//
//  }

  void render(pcl::visualization::PCLVisualizer::Ptr& viewer) const;
};

#endif //SFND_UKF_HIGHWAY_CAR_H
