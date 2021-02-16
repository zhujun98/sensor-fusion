/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */
#include "car.hpp"


Car::Car(const std::string& name, const Eigen::Vector3f& size, const Eigen::Vector3f& color)
: name_(name), size_(size), color_(color),
  pos_{0.f, 0.f, 0.f}, velocity_{25.f, 0.f, 0.f}, acc_{0.f, 0.f, 0.f} {
}

const Eigen::Vector3f& Car::position() const {
  return pos_;
}

const std::string& Car::name() const {
  return name_;
}

void Car::setColor(const Eigen::Vector3f& color) {
  for (size_t i = 0; i < 3; ++i) {
    if (color[i] > 1.f or color[i] < 0.f) {
      throw std::invalid_argument("Color values range from 0.0 to 1.0!");
    }
  }
  color_ = color;
}

void Car::moveTo(const Eigen::Vector3f& pos) {
  pos_ = pos;
}

void Car::step(float dt, size_t timestamp) {
//  if(instructions.size() > 0 && accuateIndex < (int)instructions.size()-1)
//  {
//    if(time_us >= instructions[accuateIndex+1].time_us)
//    {
//      setAcceleration(instructions[accuateIndex+1].acceleration);
//      setSteering(instructions[accuateIndex+1].steering);
//      accuateIndex++;
//    }
//  }
//
  pos_ += velocity_ * dt;
//
//  angle += velocity*steering*dt/Lf;
//  orientation = getQuaternion(angle);
//  velocity += acceleration*dt;
//
//  sinNegTheta = sin(-angle);
//  cosNegTheta = cos(-angle);
}

//void Highway::step(float dt, size_t timestamp) {
//    // Sense surrounding cars with lidar and radar
//    Eigen::VectorXd gt(4);
////    gt << car.position.x, car.position.y, traffic[i].velocity*cos(traffic[i].angle), traffic[i].velocity*sin(traffic[i].angle);
////    tools.ground_truth.push_back(gt);
////    tools.lidarSense(traffic[i], viewer, timestamp, visualize_lidar);
////    tools.radarSense(traffic[i], egoCar, viewer, timestamp, visualize_radar);
////    tools.ukfResults(traffic[i],viewer, projectedTime, projectedSteps);
////
////    Eigen::VectorXd estimate(4);
////    double v  = traffic[i].ukf.x_(2);
////    double yaw = traffic[i].ukf.x_(3);
////    double v1 = cos(yaw)*v;
////    double v2 = sin(yaw)*v;
////    estimate << traffic[i].ukf.x_[0], traffic[i].ukf.x_[1], v1, v2;
////    tools.estimations.push_back(estimate);
//  }
//}


void Car::render(pcl::visualization::PCLVisualizer::Ptr& viewer) const {
  viewer->removeShape(name_);
  viewer->removeShape(name_ + "_frame");
  viewer->removeShape(name_ + "_top");
  viewer->removeShape(name_ + "_top_frame");

  // Render the bottom of the car.
  // center, rotation, width, height, depth, name
  viewer->addCube(Eigen::Vector3f(pos_[0], pos_[1], 0.3f * size_[2]),
                  orient_, size_[0], size_[1], 0.6f * size_[2], name_);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name_);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_[0], color_[1], color_[2], name_);
  // Render the frame of the bottom of the car in black.
  viewer->addCube(Eigen::Vector3f(pos_[0], pos_[1], 0.3f * size_[2]),
                  orient_, size_[0], size_[1], 0.6f * size_[2], name_ + "_frame");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name_ + "_frame");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name_ + "_frame");

  // Render the top of the car.
  viewer->addCube(Eigen::Vector3f(pos_[0], pos_[1], 0.75f * size_[2]),
                  orient_, 0.5 * size_[0], size_[1], 0.3f * size_[2], name_ + "_top");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name_ + "_top");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_[0], color_[1], color_[2], name_ + "_top");
  // Render the frame of the bottom of the car in black.
  viewer->addCube(Eigen::Vector3f(pos_[0], pos_[1], 0.75f * size_[2]),
                  orient_, 0.5 * size_[0], size_[1], 0.3f * size_[2], name_ + "_top_frame");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name_ + "_top_frame");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name_ + "_top_frame");
}
