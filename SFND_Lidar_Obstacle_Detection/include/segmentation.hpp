/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBSTACLE_DETECTION_SEGMENTATION_H
#define LIDAR_OBSTACLE_DETECTION_SEGMENTATION_H

#include <random>

#include <pcl/common/common.h>


/**
 *  Class for single plane segmentation from a point cloud using RANSAC algorithm.
 */
template<typename T>
class SACSegmentation
{
  typename pcl::PointCloud<T>::Ptr cloud_;
  int max_iterations_;
  double threshold_;

public:

  SACSegmentation() : max_iterations_(50), threshold_(0)
  {
  };

  ~SACSegmentation() = default;

  void setMaxIterations(size_t v) { max_iterations_ = v; }

  void setDistanceThreshold(double v) { threshold_ = v; }

  void setInputCloud(const typename pcl::PointCloud<T>::Ptr& cloud) { cloud_ = cloud; }

  void segment(pcl::PointIndices& inliers, pcl::ModelCoefficients& coeff)
  {
    size_t cloud_size = cloud_->points.size();
    if (cloud_size < 3) return;

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, cloud_size - 1);

    std::vector<int> max_indices;
    std::vector<int> curr_indices;
    int iterations = 0;
    while (iterations++ < max_iterations_)
    {
      // Randomly select three points (one point could be selected more than once).
      std::vector<T> p;
      for (size_t i = 0; i < 3; ++i) p.emplace_back(cloud_->points[dis(gen)]);

      // Calculate the coefficients for the plane function ax + by + cz + d = 0.
      double a = (p[1].y - p[0].y) * (p[2].z - p[0].z) - (p[1].z - p[0].z) * (p[2].y - p[0].y);
      double b = (p[1].z - p[0].z) * (p[2].x - p[0].x) - (p[1].x - p[0].x) * (p[2].z - p[0].z);
      double c = (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[1].y - p[0].y) * (p[2].x - p[0].x);
      double d = -(a * p[0].x + b * p[0].y + c * p[0].z);
      if ( int(a == 0) + int(b == 0) + int(c == 0) >= 2 ) continue; // degeneracy

      // Find inliers.
      curr_indices.clear();
      double tol = std::sqrt(a * a + b * b + c * c) * threshold_;
      for (size_t i = 0; i < cloud_size; ++i)
      {
        auto& point = cloud_->points[i];
        if (std::abs(a * point.x + b * point.y + c * point.z + d) <= tol) curr_indices.push_back(i);
      }

      // Copy indices if a better solution was found.
      if (curr_indices.size() > max_indices.size()) max_indices = curr_indices;
    }

    inliers.indices = max_indices;
  }
};

#endif //LIDAR_OBSTACLE_DETECTION_SEGMENTATION_H
