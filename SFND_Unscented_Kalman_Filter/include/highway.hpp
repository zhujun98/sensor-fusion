/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */
#include <pcl/visualization/pcl_visualizer.h>


class Highway {

  float width_ = 12.f; // in meter
  float height_ = 0.2f; // in meter

  float x0_; // starting point of the displayed region, in meter
  float x1_; // end point of the displayed region, in meter

  float pole_space_ = 10.f;
  float pole_dist_ = 4.f;
  float pole_half_width_ = 0.25f;
  float pole_height_ = 3.f;

public:

  Highway(float start, float end);

  ~Highway() = default;

  void render(pcl::visualization::PCLVisualizer::Ptr& viewer) const;
};