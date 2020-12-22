/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include <pcl/io/pcd_io.h>

#include "streamer.hpp"


Streamer::Streamer(const std::string& path)
{
  for (auto it=boost::filesystem::directory_iterator(path);
       it != boost::filesystem::directory_iterator(); ++it)
  {
    auto p = it->path();
    if (p.extension().string() == ".pcd") paths_.emplace_back(p);
  }

  // sort files in ascending order so that playback is chronological
  std::sort(paths_.begin(), paths_.end());

  it_ = paths_.begin();
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Streamer::next()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  const std::string filename = it_->string();
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file \n");
  }

  std::cerr << "Loaded " << cloud->points.size() << " data points from " + filename << std::endl;

  // repeated streaming
  if (++it_ == paths_.end()) it_ = paths_.begin();

  return cloud;
}
