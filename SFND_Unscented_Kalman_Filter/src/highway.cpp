/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */
#include "highway.hpp"


Highway::Highway(float start, float end) {
  if (start >= end)
  {
    throw std::invalid_argument("start >= end");
  }
  x0_ = start;
  x1_ = end;
}

void Highway::render(pcl::visualization::PCLVisualizer::Ptr& viewer) const {

  float w_2 = 0.5f * width_;
  float w_6 = width_ / 6.f;

  // x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, name
  viewer->addCube(x0_, x1_, -w_2, w_2, -height_, 0.f, .2, .2, .2, "highway");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                      "highway");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highway");

  viewer->addLine(pcl::PointXYZ(x0_, -w_6, 0.01f), pcl::PointXYZ(x1_ , -w_6, 0.01f),
                  1, 1, 0, "highway_lane_line_1");
  viewer->addLine(pcl::PointXYZ(x0_, w_6, 0.01f), pcl::PointXYZ(x1_, w_6, 0.01f),
                  1, 1, 0, "highway_lane_line_2");

  // rendering poles
  float pole_x = x0_;
  size_t pole_index = 1;
  float half_road_width = 0.5f * width_;
  while(pole_x <= x1_)
  {
    //	left pole
    std::string left_pole_name = "pole_" + std::to_string(pole_index) + "l";
    viewer->addCube(-pole_half_width_ + pole_x, pole_half_width_ + pole_x,
                    -pole_half_width_ + half_road_width + pole_dist_, pole_half_width_ + half_road_width +  + pole_dist_,
                    0, pole_height_, 1, 0.5, 0, left_pole_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        left_pole_name);

    //	right pole
    std::string right_pole_name = "pole_" + std::to_string(pole_index) + "r";
    viewer->addCube(-pole_half_width_ + pole_x, pole_half_width_ + pole_x,
                    -pole_half_width_ - half_road_width - pole_dist_, pole_half_width_ - half_road_width - pole_dist_,
                    0, pole_height_, 1, 0.5, 0, right_pole_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        right_pole_name);

    pole_x += pole_space_;
    ++pole_index;
  }
}