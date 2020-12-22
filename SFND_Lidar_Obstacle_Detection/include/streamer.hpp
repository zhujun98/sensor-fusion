/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBSTACLE_DETECTION_STREAMER_HPP
#define LIDAR_OBSTACLE_DETECTION_STREAMER_HPP

#include <vector>
#include <string>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>


class Streamer
{
  std::vector<boost::filesystem::path> paths_;
  typename std::vector<boost::filesystem::path>::iterator it_;

public:

  /**
   * Constructor.
   *
   * @param path: Path of the data folder.
   */
  explicit Streamer(const std::string& path);

  ~Streamer() = default;

  /**
   * Return the next point cloud in the stream.
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr next();
};

#endif //LIDAR_OBSTACLE_DETECTION_STREAMER_HPP
