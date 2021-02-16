/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */
#include "highway.hpp"
#include "car.hpp"
#include "meter.hpp"


enum class CameraAngle {
  XY, TopDown, Side, FPS
};


void initCamera(CameraAngle angle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();

  int distance = 16;

  switch(angle)
  {
    case CameraAngle::XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case CameraAngle::TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case CameraAngle::Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case CameraAngle::FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(angle != CameraAngle::FPS) viewer->addCoordinateSystem(1.0);
}

std::vector<Car> initTraffic() {
  std::vector<Car> traffic;
  for (size_t i = 0; i < 3; ++i) {
    Car car("car_" + std::to_string(i));
    car.setColor({0.f, 0.f, 1.f});
    traffic.emplace_back(std::move(car));
  }
  traffic[0].moveTo({-10., 4., 0.});
  traffic[1].moveTo({25, -4, 0});
  traffic[2].moveTo({-12, 0, 0});

  return traffic;
}

//  egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
//
//  std::vector<accuation> car1_instructions;
//  accuation a = accuation(0.5*1e6, 0.5, 0.0);
//  car1_instructions.push_back(a);
//  a = accuation(2.2*1e6, 0.0, -0.2);
//  car1_instructions.push_back(a);
//  a = accuation(3.3*1e6, 0.0, 0.2);
//  car1_instructions.push_back(a);
//  a = accuation(4.4*1e6, -2.0, 0.0);
//  car1_instructions.push_back(a);
//
//  car1.setInstructions(car1_instructions);

//
//  Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
//  std::vector<accuation> car2_instructions;
//  a = accuation(4.0*1e6, 3.0, 0.0);
//  car2_instructions.push_back(a);
//  a = accuation(8.0*1e6, 0.0, 0.0);
//  car2_instructions.push_back(a);
//  car2.setInstructions(car2_instructions);

//
//  Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
//  std::vector<accuation> car3_instructions;
//  a = accuation(0.5*1e6, 2.0, 1.0);
//  car3_instructions.push_back(a);
//  a = accuation(1.0*1e6, 2.5, 0.0);
//  car3_instructions.push_back(a);
//  a = accuation(3.2*1e6, 0.0, -1.0);
//  car3_instructions.push_back(a);
//  a = accuation(3.3*1e6, 2.0, 0.0);
//  car3_instructions.push_back(a);
//  a = accuation(4.5*1e6, 0.0, 0.0);
//  car3_instructions.push_back(a);
//  a = accuation(5.5*1e6, -2.0, 0.0);
//  car3_instructions.push_back(a);
//  a = accuation(7.5*1e6, 0.0, 0.0);
//  car3_instructions.push_back(a);
//  car3.setInstructions(car3_instructions);

//
//  lidar = new Lidar(traffic,0);

int main(int argc, char** argv) {

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  initCamera(CameraAngle::XY, viewer);

  // initialization
	Highway highway(-50, 450);
	Car ego_car("car_ego");
  std::vector<Car> traffic = initTraffic();
  std::vector<double> rmse_threshold = {0.30, 0.16, 0.95, 0.70};
  Meter meter;

  float dt = 1.f / 30.f; // 30 frames/s
  size_t time_us = 0;

  highway.render(viewer);
  for (size_t i = 0; i < 300; ++i) {
    viewer->removeAllPointClouds();

    // updating
    ego_car.step(dt, time_us);
    for (auto& v : traffic) {
      v.step(dt, time_us);
    }

    // rendering
    ego_car.render(viewer);
    for (const auto& v : traffic) {
      v.render(viewer);
    }

    time_us += 1e6 * dt;

    viewer->spinOnce(20);
  }
}