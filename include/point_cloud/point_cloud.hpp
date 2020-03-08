#ifndef POINT_CLOUD_HPP_
#define POINT_CLOUD_HPP_

#include <memory>
#include <vector>

namespace point_cloud {
//基础点云
template <typename PointT>
class PointCloud {
 public:
  PointCloud() { point_cloud_.reset(new std::vector<PointT>); }
  PointCloud(const PointCloud& rhs) = default;
  PointCloud& operator=(const PointCloud& rhs) = default;
  virtual ~PointCloud() = default;

 protected:
  std::unique_ptr<std::vector<PointT>> point_cloud_;  //点云数据

 public:
  //获得点云数据
  const std::vector<PointT>& pointCloud() const { return *point_cloud_; }

  //获得序数对应的点
  const PointT& operator[](const size_t index) const {
    return (*point_cloud_)[index];
  }

  //点数量
  const size_t size() const { return kdtree_get_point_count(); }

  //设置点云
  virtual void set(const std::vector<PointT>& point_cloud) {
    point_cloud_->clear();
    add(point_cloud);
  }

  //添加新的点云到原来的点云中
  virtual void add(const std::vector<PointT>& point_cloud) {
    point_cloud_->insert(point_cloud_->end(), point_cloud.begin(),
                         point_cloud.end());
  }

  //添加新的点到原来的点云中
  virtual void add(const PointT& point) { point_cloud_->push_back(point); }

  //清空点云数据
  virtual void clear() { point_cloud_->clear(); }

  //辅助接口函数,用于nanoflann库,使得可以建立kd树
  size_t kdtree_get_point_count() const { return point_cloud_->size(); }

  //辅助接口函数,用于nanoflann库,使得可以建立kd树
  float kdtree_get_pt(const size_t index, int dim) const {
    const PointT& p = (*point_cloud_)[index];
    if (dim == 0) {
      return p.x;
    } else if (dim == 1) {
      return p.y;
    } else if (dim == 2) {
      return p.z;
    }
    return 0.f;
  }

  //辅助接口函数,用于nanoflann库,使得可以建立kd树
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};
}  // namespace point_cloud

#endif