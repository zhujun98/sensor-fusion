#include <cassert>
#include <random>

#include "../include/cluster.hpp"


typename pcl::PointCloud<pcl::PointXYZI>::Ptr createPointCloud()
{
  typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dis(0, 0.1);

  // cluster 1
  for (size_t i = 0; i < 3; ++i)
  {
    auto pt = pcl::PointXYZI();
    pt.x = 2.f + dis(gen);
    pt.y = 2.f + dis(gen);
    pt.z = 4.f + dis(gen);
    cloud->points.emplace_back(pt);
  }

  // cluster 2
  for (size_t i = 0; i < 5; ++i)
  {
    auto pt = pcl::PointXYZI();
    pt.x = 1.f + dis(gen);
    pt.y = 1.f + dis(gen);
    pt.z = 1.f + dis(gen);
    cloud->points.emplace_back(pt);
  }

  // cluster 3
  for (size_t i = 0; i < 4; ++i)
  {
    auto pt = pcl::PointXYZI();
    pt.x = 5.f + dis(gen);
    pt.y = 4.f + dis(gen);
    pt.z = 3.f + dis(gen);
    cloud->points.emplace_back(pt);
  }

  // cluster 3
  for (size_t i = 0; i < 2; ++i)
  {
    auto pt = pcl::PointXYZI();
    pt.x = 10.f + dis(gen);
    pt.y = 10.f + dis(gen);
    pt.z = 10.f + dis(gen);
    cloud->points.emplace_back(pt);
  }

  return cloud;
}

void testBuildKdTree()
{
  auto cloud = createPointCloud();
  auto tree = KdTree<pcl::PointXYZI>();
  tree.setInputCloud(cloud);

  auto& points = cloud->points;
  auto root = tree.root();

  // depth == 0
  auto left = root->left;
  auto right = root->right;
  assert(points[left->index].x <= points[root->index].x);
  assert(points[right->index].x >= points[root->index].x);

  // depth == 1
  auto lleft = left->left;
  auto lright = left->right;
  if (lleft != nullptr) assert(points[lleft->index].y <= points[left->index].y);
  if (lright != nullptr) assert(points[lright->index].y >= points[left->index].y);

  auto rleft = right->left;
  auto rright = right->right;
  if (rleft != nullptr) assert(points[rleft->index].y <= points[right->index].y);
  if (rright != nullptr) assert(points[rright->index].y >= points[right->index].y);

  // depth == 2
  auto llleft = lleft->left;
  auto llright = lleft->right;
  if (llleft != nullptr) assert(points[llleft->index].z <= points[lleft->index].z);
  if (llright != nullptr) assert(points[llright->index].z >= points[lleft->index].z);
}

void testSearchNeighbors()
{
  auto cloud = createPointCloud();
  std::shared_ptr<KdTree<pcl::PointXYZI>> kd_tree(new KdTree<pcl::PointXYZI>());
  EuclideanClusterExtraction<pcl::PointXYZI> ec;

  kd_tree->setInputCloud(cloud);

  ec.setClusterTolerance(1.);
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(100);
  ec.setSearchMethod(kd_tree);
  ec.setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  assert(cluster_indices.size() == 4);
  for (auto& indices : cluster_indices)
  {
    for (auto index : indices.indices)
    {
      std::cout << cloud->points[index] << ", ";
    }
    std::cout << "\n";
  }
  std::cout << std::endl;

  ec.setMinClusterSize(3);
  cluster_indices.clear();
  ec.extract(cluster_indices);
  assert(cluster_indices.size() == 3);

  ec.setMaxClusterSize(4);
  cluster_indices.clear();
  ec.extract(cluster_indices);
  assert(cluster_indices.size() == 2);

  ec.setMinClusterSize(10);
  cluster_indices.clear();
  ec.extract(cluster_indices);
  assert(cluster_indices.empty());
}

int main(int argc, char** argv)
{
  testBuildKdTree();
  testSearchNeighbors();

  return 0;
}
