/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBSTACLE_DETECTION_PROCESS_HPP
#define LIDAR_OBSTACLE_DETECTION_PROCESS_HPP

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#if !defined(USE_PCL_SEG)
#include "segmentation.hpp"
#endif

/**
 * Reduce the number of points by applying filters.
 *
 * VoxelGrid filter -> CropBox filter
 *
 * @param cloud: Input point cloud.
 * @param leaf_size: Leaf size used in VoxelGrid filter, in meter.
 * @param x_min: Minimum x of the ROI, in meter.
 * @param x_max: Maximum x of the ROI, in meter
 * @param y_min: Minimum y of the ROI, in meter
 * @param y_max: Maximum y of the ROI, in meter
 * @param z_min: Minimum z of the ROI, in meter
 * @param z_max: Maximum z of the ROI, in meter
 */
template<typename T>
void filterCloud(typename pcl::PointCloud<T>::Ptr cloud, float leaf_size,
                 float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
  auto startTime = std::chrono::steady_clock::now();

  // reduce the number points by applying a voxel grid filter
  pcl::VoxelGrid<T> vg_filter;
  vg_filter.setInputCloud(cloud);
  vg_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg_filter.filter(*cloud);

  // remove the point cloud outside the region of interest
  pcl::CropBox<T> cb_filter;
  cb_filter.setMin({x_min, y_min, z_min, 1.});
  cb_filter.setMax({x_max, y_max, z_max, 1.});
  cb_filter.setInputCloud(cloud);
  cb_filter.filter(*cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
}


/**
 * Segment point cloud into two parts: the ground plane and the obstacles.
 *
 * @param cloud: Input point cloud.
 * @param max_iters: Maximum number of iterations allowed in the RANSAC algorithm.
 * @param threshold: Distance threshold used in SACMODEL_PLANE.
 *
 * @return: The point cloud of the obstacles.
 */
template<typename T>
typename pcl::PointCloud<T>::Ptr
segmentCloud(typename pcl::PointCloud<T>::Ptr cloud, int max_iters, float threshold)
{
  auto startTime = std::chrono::steady_clock::now();

  // use RANSAC to find out the ground plane

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

#if defined(USE_PCL_SEG)
  pcl::SACSegmentation<T> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
#else
  SACSegmentation<T> seg;
#endif
  seg.setMaxIterations(max_iters);
  seg.setDistanceThreshold(threshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // separate the original cloud into ground plane and obstacles

  pcl::ExtractIndices<T> et_filter;
  et_filter.setIndices(inliers);
  et_filter.setInputCloud(cloud);

  typename pcl::PointCloud<T>::Ptr cloud_obstacles(new pcl::PointCloud<T>);
  et_filter.setNegative(true);
  et_filter.filter(*cloud_obstacles);

  et_filter.setNegative(false);
  et_filter.filter(*cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloud_obstacles;
}


/**
 * Cluster point cloud.
 *
 * @param cloud: Input point cloud.
 * @param tolerance: The spatial cluster tolerance as a measure in the L2 Euclidean space.
 * @param min_size: The minimum number of points that a cluster needs to contain in order to be considered valid.
 * @param max_size: The maximum number of points that a cluster needs to contain in order to be considered valid.
 * @return: A vector of clustered point clouds.
 */
template<typename T>
std::vector<typename pcl::PointCloud<T>::Ptr>
clusterCloud(typename pcl::PointCloud<T>::Ptr cloud, float tolerance, int min_size, int max_size)
{
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<T>::Ptr> clusters;

  typename pcl::search::KdTree<T>::Ptr kd_tree(new pcl::search::KdTree<T>);
  kd_tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<T> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(kd_tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (const auto& indices : cluster_indices)
  {
    typename pcl::PointCloud<T>::Ptr cluster(new pcl::PointCloud<T>());
    for (const auto& idx : indices.indices)
    {
      cluster->push_back((*cloud)[idx]);
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;
    }

    clusters.push_back(cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Clustering took " << elapsedTime.count() << " milliseconds\n";
  std::cout << "Found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

#endif //LIDAR_OBSTACLE_DETECTION_PROCESS_HPP
