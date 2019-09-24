#include "render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processor.hpp"
#include "pcd_streamer.hpp"
#include "config.hpp"


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();

  int distance = CAMERA_DISTANCE;

  switch(setAngle)
  {
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(setAngle != FPS) viewer->addCoordinateSystem(1.0);
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

    // process the point cloud

    filterCloud<pcl::PointXYZI>(cloud, 0.1f);

    renderPointCloud(viewer, cloud, filename);

    // repeated streaming
    if (++it == streamer.end()) it = streamer.begin();
    viewer->spinOnce();
  }
}
