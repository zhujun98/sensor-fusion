/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBSTACLE_DETECTION_PCD_STREAMER_HPP
#define LIDAR_OBSTACLE_DETECTION_PCD_STREAMER_HPP

#include <vector>
#include <string>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>


class PcdStreamer
{
  boost::filesystem::path path_;
  std::vector<boost::filesystem::path> pcd_paths_;

public:

  explicit PcdStreamer(const std::string& data_path) : path_(data_path)
  {
    try
    {
      if (exists(path_))    // does p actually exist?
      {
        if (is_directory(path_))      // is p a directory?
        {
          for (auto it=boost::filesystem::directory_iterator(path_);
               it != boost::filesystem::directory_iterator(); ++it)
          {
            auto p = it->path();
            if (p.extension().string() == ".pcd") pcd_paths_.emplace_back(p);
          }

          // sort files in ascending order so that playback is chronological
          std::sort(pcd_paths_.begin(), pcd_paths_.end());
        } else
        {
          cout << path_ << " exists, but is not a directory\n";
        }
      }
      else
        cout << path_ << " does not exist\n";

    } catch (const boost::filesystem::filesystem_error& ex)
    {
      cout << ex.what() << '\n';
    }
  }

  ~PcdStreamer() = default;

  using const_iterator = std::vector<boost::filesystem::path>::const_iterator;

  const_iterator begin() const noexcept { return pcd_paths_.cbegin(); }
  const_iterator end() const noexcept { return pcd_paths_.cend(); }
};

#endif //LIDAR_OBSTACLE_DETECTION_PCD_STREAMER_HPP
