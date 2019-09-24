#ifndef LIDAR_OBJECT_DETECTION_RENDER_HPP
#define LIDAR_OBJECT_DETECTION_RENDER_HPP

#include <iostream>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>


struct Color
{
  float r, g, b;

  Color(float setR, float setG, float setB)
    : r(setR), g(setG), b(setB)
  {}
};


enum CameraAngle
{
  XY, TopDown, Side, FPS
};


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      const std::string& name,
                      Color color = Color(-1, -1, -1));

#endif // LIDAR_OBJECT_DETECTION_RENDER_HPP
