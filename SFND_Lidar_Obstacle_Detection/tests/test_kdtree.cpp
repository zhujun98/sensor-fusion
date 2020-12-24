#include <cassert>

#include "../include/cluster.hpp"


void testBuildKdTree()
{
  auto tree = KdTree<pcl::PointXYZI>();
  typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < 10; ++i)
  {
    auto pt = pcl::PointXYZI();
    cloud->points.emplace_back(pt);
  }
  tree.setInputCloud(cloud);

  auto root = tree.root();
  assert(root->index == 0);

  auto next = root;
  assert(root->left == nullptr);
  assert(next->right->index == 1);
  next = root->right;

  assert(root->left == nullptr);
  assert(next->right->index == 2);
}

void testSearchNeighbors()
{

}

int main(int argc, char** argv)
{
  testBuildKdTree();
  testSearchNeighbors();

  return 0;
}
