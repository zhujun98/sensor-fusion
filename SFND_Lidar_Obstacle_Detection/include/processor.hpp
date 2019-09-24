/*
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBSTACLE_DETECTION_PROCESSOR_HPP
#define LIDAR_OBSTACLE_DETECTION_PROCESSOR_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>


/**
 * Reduce the number of points by applying filters.
 *
 * VoxelGrid filter -> CropBox filter
 *
 * @param cloud: the input point cloud.
 * @param leaf_size: leaf size used in VoxelGrid filter, in meter.
 * @param x_min: minimum x of the ROI, in meter.
 * @param x_max: maximum x of the ROI, in meter
 * @param y_min: minimum y of the ROI, in meter
 * @param y_max: maximum y of the ROI, in meter
 * @param z_min: minimum z of the ROI, in meter
 * @param z_max: maximum z of the ROI, in meter
 */
template<typename PointT>
void filterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float leaf_size,
                 float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
  auto startTime = std::chrono::steady_clock::now();

  // reduce the number points by applying a voxel grid filter
  typename pcl::VoxelGrid<PointT> vg_filter;
  vg_filter.setInputCloud(cloud);
  vg_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg_filter.filter(*cloud);

  // remove the point cloud outside the region of interest
  typename pcl::CropBox<PointT> cb_filter;
  cb_filter.setMin({x_min, y_min, z_min, 1.});
  cb_filter.setMax({x_max, y_max, z_max, 1.});
  cb_filter.setInputCloud(cloud);
  cb_filter.filter(*cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
}


/**
 * Segment cloud into two parts: ground plane and obstacle.
 *
 * @param cloud: the input point cloud.
 * @param max_iters: maximum number of iterations allowed in the RANSAC algorithm.
 * @param threshold: distance threshold used in SACMODEL_PLANE.
 *
 * @return: the point cloud of the obstacles.
 */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr segmentCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                   int max_iters,
                                                   float threshold)
{
  auto startTime = std::chrono::steady_clock::now();

  // use RANSAC to find out the ground plane

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  typename pcl::SACSegmentation<PointT> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iters);
  seg.setDistanceThreshold(threshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);


  // separate the original cloud into ground plane and obstacles

  typename pcl::ExtractIndices<PointT> et_filter;
  et_filter.setIndices(inliers);
  et_filter.setInputCloud(cloud);
  typename pcl::PointCloud<PointT>::Ptr cloud_obstacles(new pcl::PointCloud<PointT>);
  et_filter.setNegative(true);
  et_filter.filter(*cloud_obstacles);

  et_filter.setNegative(false);
  et_filter.filter(*cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloud_obstacles;
}


#endif //LIDAR_OBSTACLE_DETECTION_PROCESSOR_HPP
