#ifndef KD_TREE_HPP_
#define KD_TREE_HPP_

#include <memory>
#include <vector>
#include "nanoflann.hpp"

namespace kdtree {
// kd树基类(包含kd树的一些基本操作)
template <typename PointT, template <typename> class PointCloudT>
class KdTree {
 public:
  explicit KdTree() {
    params_.sorted = true;
    point_cloud_.reset(new PointCloudT<PointT>);
  }
  virtual ~KdTree() = default;

 protected:
  std::unique_ptr<PointCloudT<PointT>> point_cloud_;  //原始点云
  nanoflann::SearchParams params_;                    //搜索参数

 public:
  //获得原始点云
  const std::vector<PointT>& pointCloud() const { return *point_cloud_; }

  //获得序数对应的点
  const PointT& operator[](const size_t index) const {
    return (*point_cloud_)[index];
  }

  //重置整个kd树
  virtual void reset() = 0;

  //搜索最近的k个点
  virtual int nearestKSearch(const PointT& point, int num_closest,
                             std::vector<int>& k_indices,
                             std::vector<float>& k_sqr_distances) const = 0;

  //搜索固定半径的圆内的点
  virtual int radiusSearch(const PointT& point, float radius,
                           std::vector<int>& k_indices,
                           std::vector<float>& k_sqr_distances) const = 0;
};
}  // namespace kdtree
#endif