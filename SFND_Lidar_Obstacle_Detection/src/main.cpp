/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include "render.hpp"
#include "process.hpp"
#include "streamer.hpp"


enum class CameraAngle
{
  XY, TopDown, Side, FPS
};


void initCamera(CameraAngle angle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
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


int main (int argc, char** argv)
{
  std::string usage_instructions = "Usage: ";
  usage_instructions += argv[0];
  usage_instructions += " data_folder";

  if (argc != 2) {
    std::cerr << usage_instructions << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string data_folder = argv[1];

  std::cout << "starting environment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  initCamera(CameraAngle::XY, viewer);

  Streamer streamer(data_folder);

  while (!viewer->wasStopped())
  {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes(); // remove bounding boxes

    auto cloud = streamer.next();

//    renderPointCloud(viewer, cloud, filename); // visualize the raw point cloud

    // Downsampling by applying vortex grid filter and region-of-interest filter.
    filterCloud<pcl::PointXYZI>(cloud, 0.2, -20, 40, -6.5, 6.5, -2, 2);

    // Separate the filtered point cloud into two parts: ground plane and obstacles.
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_clouds = segmentCloud<pcl::PointXYZI>(cloud, 100, 0.2);

    // Cluster the obstacles.
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
      obstacle_clusters = clusterCloud<pcl::PointXYZI>(obstacle_clouds, 0.5, 50, 500);

    // Render the ground and the clustered obstacles.
    renderPointCloud<pcl::PointXYZI>(viewer, cloud, "ground", CloudColor::ROAD);
    size_t count = 0;
    for (const auto& cluster : obstacle_clusters)
    {
      renderPointCloud<pcl::PointXYZI>(viewer, cluster, "obstacle_" + std::to_string(++count), CloudColor::OBSTACLE);
    }

    viewer->spinOnce();
  }
}
