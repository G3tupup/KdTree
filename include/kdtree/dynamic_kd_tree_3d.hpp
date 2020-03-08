#ifndef DYNAMIC_KD_TREE_3D_HPP_
#define DYNAMIC_KD_TREE_3D_HPP_

#include "kd_tree.hpp"

namespace kdtree {
//动态三维搜索kd树
template <typename PointT, template <typename> class PointCloudT>
class DynamicKdTree3d : public KdTree<PointT, PointCloudT> {
 public:
  explicit DynamicKdTree3d() {
    dynamic_kd_tree_base_.reset(new KdTreeBaseT(3, *point_cloud_));
  }

 private:
  using KdTree<PointT, PointCloudT>::point_cloud_;
  using KdTree<PointT, PointCloudT>::params_;
  using KdTreeBaseT = nanoflann::KDTreeSingleIndexDynamicAdaptor<
      nanoflann::SO3_Adaptor<float, PointCloudT<PointT>>, PointCloudT<PointT>,
      3>;
  std::unique_ptr<KdTreeBaseT> dynamic_kd_tree_base_;

 public:
  //动态添加新的点云到kd树中
  void add(const std::vector<PointT>& point_cloud) {
    if (point_cloud.size() > 0) {
      const auto size = point_cloud_->size();
      point_cloud_->add(point_cloud);
      dynamic_kd_tree_base_->addPoints(size, point_cloud_->size() - 1);
    } else {
      std::cout << "None Points added!" << std::endl;
    }
  }

  void reset() override {
    point_cloud_->clear();
    dynamic_kd_tree_base_.reset(new KdTreeBaseT(3, *point_cloud_));
  }

  int nearestKSearch(const PointT& point, int num_closest,
                     std::vector<int>& k_indices,
                     std::vector<float>& k_sqr_distances) const override {
    k_indices.resize(num_closest);
    k_sqr_distances.resize(num_closest);
    nanoflann::KNNResultSet<float, int> resultSet(num_closest);
    resultSet.init(k_indices.data(), k_sqr_distances.data());
    dynamic_kd_tree_base_->findNeighbors(resultSet, point.data, params_);
    return resultSet.size();
  }

  int radiusSearch(const PointT& point, float radius,
                   std::vector<int>& k_indices,
                   std::vector<float>& k_sqr_distances) const override {
    std::vector<std::pair<int, float>> indices_dist;
    indices_dist.reserve(128);
    nanoflann::RadiusResultSet<float, int> resultSet(radius * radius,
                                                     indices_dist);
    dynamic_kd_tree_base_->findNeighbors(resultSet, point.data, params_);
    const size_t nFound = resultSet.size();
    if (params_.sorted) {
      std::sort(indices_dist.begin(), indices_dist.end(),
                nanoflann::IndexDist_Sorter());
    }
    k_indices.resize(nFound);
    k_sqr_distances.resize(nFound);
    for (int i = 0; i < nFound; i++) {
      k_indices[i] = indices_dist[i].first;
      k_sqr_distances[i] = indices_dist[i].second;
    }
    return nFound;
  }
};
}  // namespace kdtree

#endif