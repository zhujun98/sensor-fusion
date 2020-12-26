/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBJECT_DETECTION_RENDER_HPP
#define LIDAR_OBJECT_DETECTION_RENDER_HPP

#include <iostream>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>


enum class CloudColor
{
  INTENSITY, OBSTACLE, ROAD
};


/**
 * Render the point cloud in the viewer.
 *
 * @param viewer: PCLVisualizer pointer.
 * @param cloud: Input point cloud.
 * @param name: Point cloud name.
 * @param color: Color scheme for the point cloud.
 */
template<typename T>
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const typename pcl::PointCloud<T>::Ptr& cloud,
                      const std::string& name,
                      CloudColor color = CloudColor::INTENSITY)
{
  if (color == CloudColor::INTENSITY)
  {
    pcl::visualization::PointCloudColorHandlerGenericField<T> intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<T>(cloud, intensity_distribution, name);
  } else
  {
    double r = 0., g = 1., b = 1.; // default color (cyan) for CloudColor::ROAD
    if (color == CloudColor::OBSTACLE)
    {
      T min_pt, max_pt;
      pcl::getMinMax3D(*cloud, min_pt, max_pt);

      // Set the obstacle color by the height.
      if ( (max_pt.x - min_pt.x < 0.5) & (max_pt.y - min_pt.y < 0.5) )
      {
        // pole
        r = 1., g = 1., b = 0.;
      } else if (max_pt.z - min_pt.z > 1.5)
      {
        // big car
        r = 0., g = 0., b = 1.;
      } else {
        // small car
        r = 1., g = 0., b = 0.;
      }

      // Draw a yellow bounding box.
      std::string box_name = name + "_box";
      viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, 0.0, 1.0, 0.0, box_name);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                          box_name);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1., box_name);
    }
    viewer->addPointCloud<T>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
  }

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

#endif // LIDAR_OBJECT_DETECTION_RENDER_HPP
