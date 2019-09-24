/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#include "render.h"
#include "processor.hpp"
#include "pcd_streamer.hpp"
#include "config.hpp"


void initCamera(CameraAngle angle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();

  int distance = CAMERA_DISTANCE;

  switch(angle)
  {
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(angle != FPS) viewer->addCoordinateSystem(1.0);
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

  PcdStreamer streamer(data_folder);

  auto it = streamer.begin();

  while (!viewer->wasStopped())
  {
    viewer->removeAllPointClouds();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(it->string(), *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
    }

    std::string filename = it->filename().string();
    std::cerr << "Loaded " << cloud->points.size () << " data points from " + filename << std::endl;

//    renderPointCloud(viewer, cloud, filename); // visualize the raw point cloud

    // process the point cloud

    // reduce the number cloud by applying vortex grid filter and Region-of-interest filter
    float leaf_size = 0.2;
    float x_min = -20.;
    float x_max =  40.;
    float y_min = -10.;
    float y_max =  10.;
    float z_min =  -2.;  // should cover the ground plan
    float z_max =   2.;
    filterCloud<pcl::PointXYZI>(cloud, leaf_size, x_min, x_max, y_min, y_max, z_min, z_max);

    // separate the rest point cloud into two parts: ground plane and obstacles
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacles = segmentCloud<pcl::PointXYZI>(cloud, 100, 0.2);

    // colorize the ground plane and obstacles
    renderPointCloud(viewer, cloud, "ground", Color(0, 1, 1));
    renderPointCloud(viewer, cloud_obstacles, "obstacles", Color(1, 1, 0));

    // repeated streaming
    if (++it == streamer.end()) it = streamer.begin();
    viewer->spinOnce();
  }
}
