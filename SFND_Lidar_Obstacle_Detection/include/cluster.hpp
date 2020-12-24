/**
 * Author: Jun Zhu, zhujun981661@gmail.com
 */

#ifndef LIDAR_OBSTACLE_DETECTION_CLUSTER_H
#define LIDAR_OBSTACLE_DETECTION_CLUSTER_H

#include <iostream>
#include <queue>

#include <pcl/common/common.h>


struct TreeNode
{
  size_t index;

  TreeNode* left;
  TreeNode* right;

  explicit TreeNode(int idx) : index(idx), left(nullptr), right(nullptr) {}

  ~TreeNode()
  {
    delete left;
    delete right;
  }
};


template<typename T>
class KdTree
{
  typename pcl::PointCloud<T>::Ptr cloud_;
  TreeNode* root_;
  size_t n_dims_;

  /**
   * Return the difference between two nodes at a given depth.
   */
  float diff(TreeNode* node, TreeNode* root, size_t depth) const
  {
    auto& points = cloud_->points;
    size_t curr_dim = depth % n_dims_;
    switch (curr_dim)
    {
      case 0: return points[node->index].x - points[root->index].x;
      case 1: return points[node->index].y - points[root->index].y;
      case 2: return points[node->index].z - points[root->index].z;
      default: throw std::out_of_range("");
    }
  }

  /**
   * Iteratively search for the leaf to insert the new node.
   *
   * @param root: Current node.
   * @param node: New node to be inserted.
   * @param depth: Depth of the root.
   */
  void insertImp(TreeNode* root, TreeNode* node, size_t depth)
  {
    if (diff(node, root, depth) < 0)
    {
      if (root->left == nullptr)
      {
        root->left = node;
      } else
      {
        insertImp(root->left, node, depth+1);
      }
    } else
    {
      if (root->right == nullptr)
      {
        root->right = node;
      } else
      {
        insertImp(root->right, node, depth+1);
      }
    }
  }

  using Neighbors = std::vector<size_t>;

  float distance(const T& p1, const T& p2) const
  {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void searchNeighborsImp(const T& point,
                          TreeNode* node,
                          float tolerance,
                          size_t depth,
                          Neighbors& neighbors) const
  {
    if (node == nullptr) return;

    size_t index = node->index;
    auto& node_point = cloud_->points[index];
    // Add the current node if the distance is within the tolerance.
    if (distance(point, node_point) <= tolerance) neighbors.push_back(index);

    // Decide whether to continue search the left and/or right branch of the tree.
    bool search_left = true;
    bool search_right = true;
    size_t curr_dim = depth % n_dims_;
    switch (curr_dim)
    {
      case 0: if (node_point.x < point.x - tolerance) search_left = false;
        if (node_point.x > point.x + tolerance) search_right = false;
        break;
      case 1: if (node_point.y < point.y - tolerance) search_left = false;
        if (node_point.y > point.y + tolerance) search_right = false;
        break;
      case 2: if (node_point.z < point.z - tolerance) search_left = false;
        if (node_point.z > point.z + tolerance) search_right = false;
        break;
      default: throw std::out_of_range("");
    }

    if (search_left) searchNeighborsImp(point, node->left, tolerance, depth+1, neighbors);
    if (search_right) searchNeighborsImp(point, node->right, tolerance, depth+1, neighbors);
  }

public:

  KdTree() : root_(nullptr), n_dims_(3)
  {
  }

  ~KdTree()
  {
    delete root_;
  }

  /**
   * Insert a node with value v into the tree.
   *
   * @param idx: Index of cloud point.
   */
  void insert(int idx)
  {
    auto node = new TreeNode(idx);
    if (root_ == nullptr)
    {
      root_ = node;
    } else
    {
      insertImp(root_, node, 0);
    }
  }

  /**
   * Construct a kd-tree from a point cloud.
   *
   * @param cloud: Input point cloud.
   */
  void setInputCloud(const typename pcl::PointCloud<T>::Ptr& cloud)
  {
    cloud_ = cloud;
    for (size_t i = 0; i < cloud->size(); ++i) insert(i);
  }

  /**
   * Return the root node (for unittest only).
   */
  TreeNode* root() const { return root_; }

  /**
   * Search for neighbors of the given point within the distance tolerance.
   *
   * @param point: Cloud point.
   * @param tolerance: Distance tolerance.
   * @return: A set of indices of the neighbors.
   */
  Neighbors searchNeighbors(const T& point, float tolerance)
  {
    Neighbors neighbors;

    searchNeighborsImp(point, root_, tolerance, 0, neighbors);

    return neighbors;
  }

};


/**
 *  Class for Euclidean clustering of a point cloud.
 */
template<typename T>
class EuclideanClusterExtraction
{
  float tolerance_;
  size_t min_size_;
  size_t max_size_;

  typename pcl::PointCloud<T>::Ptr cloud_;
  std::shared_ptr<KdTree<T>> tree_;

public:

  EuclideanClusterExtraction()
    : tolerance_(0), min_size_(1), max_size_(std::numeric_limits<int>::max())
  {
  }

  ~EuclideanClusterExtraction() = default;

  void setClusterTolerance(float tolerance) { tolerance_ = tolerance; }

  void setMinClusterSize(size_t min_size) { min_size_ = min_size; }

  void setMaxClusterSize(size_t max_size) { max_size_ = max_size; }

  void setSearchMethod(const std::shared_ptr<KdTree<T>>& tree) { tree_ = tree; }

  void setInputCloud(const typename pcl::PointCloud<T>::Ptr& cloud) { cloud_ = cloud; }

  /**
   * Extract clustered indices.
   *
   * @param cluster_indices: A vector of clustered indices.
   */
  void extract(std::vector<pcl::PointIndices>& cluster_indices)
  {
    size_t cloud_size = cloud_->size();
    // Track visited indices.
    std::vector<bool> visited(cloud_size, false);
    // Current unprocessed queue.
    std::queue<int> unprocessed;
    for (size_t i = 0; i < cloud_size; ++i)
    {
      if (visited[i]) continue;

      pcl::PointIndices point_indices; // store indices of the current cluster
      auto& indices = point_indices.indices;

      // Carry out nearest neighbor search until the current unprocessed queue is empty.
      unprocessed.push(i);
      while (!unprocessed.empty())
      {
        size_t curr_idx = unprocessed.front();
        // curr_idx is included in neighbors.
        auto neighbors = tree_->searchNeighbors(cloud_->points[curr_idx], tolerance_);
        unprocessed.pop();

        for (auto nb_idx : neighbors)
        {
          if (!visited[nb_idx])
          {
            indices.push_back(nb_idx);
            unprocessed.push(nb_idx);
            visited[nb_idx] = true;
          }
        }
      }

      // Add the current cluster to the vector.
      size_t cluster_size = indices.size();
      if ( (cluster_size >= min_size_) && (cluster_size <= max_size_) )
      {
        cluster_indices.push_back(point_indices);
      }
    }
  }
};

#endif //LIDAR_OBSTACLE_DETECTION_CLUSTER_H
