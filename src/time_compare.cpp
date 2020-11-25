#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <cstdlib>

#include "../include/kdtree/static_3d_tree.hpp"
#include "../include/point_cloud/point_cloud.hpp"
#include "../include/point_type/point_type.hpp"
#include "../include/time/time_counter.hpp"

float random_0_10() {
  return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 10.f;
}

int main() {
  for (int n = 0; n < 10; n++) {
    pcl::KdTreeFLANN<pcl::PointXYZ> pcl_kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    kd_tree::Static3dTree<point_cloud::PointCloud<point_type::Point3f>>
        static_3d_tree;
    std::vector<point_type::Point3f> points;
    points.reserve(100000);
    for (int i = 0; i < 100000; i++) {
      float x = random_0_10();
      float y = random_0_10();
      float z = random_0_10();
      points.emplace_back(x, y, z);
      pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
    }
    time_counter::tick("Static3dTree buildtree");
    static_3d_tree.build(points);
    time_counter::tock("Static3dTree buildtree");
    time_counter::tick("KdTreeFLANN buildtree");
    pcl_kdtree.setInputCloud(pcl_cloud);
    time_counter::tock("KdTreeFLANN buildtree");
    for (int i = 0; i < 100; i++) {
      float x = random_0_10();
      float y = random_0_10();
      float z = random_0_10();
      point_type::Point3f point_to_search(x, y, z);
      pcl::PointXYZ pcl_point_to_search(x, y, z);
      std::vector<size_t> indices;
      std::vector<float> squared_distances;
      std::vector<int> pcl_indices;
      std::vector<float> pcl_squared_distances;
      time_counter::tick("Static3dTree nearestKSearch");
      size_t num_3d = static_3d_tree.nearestKSearch(point_to_search, 6, indices,
                                                    squared_distances);
      time_counter::tock("Static3dTree nearestKSearch");

      time_counter::tick("KdTreeFLANN nearestKSearch");
      int pcl_num_3d = pcl_kdtree.nearestKSearch(
          pcl_point_to_search, 6, pcl_indices, pcl_squared_distances);
      time_counter::tock("KdTreeFLANN nearestKSearch");

      time_counter::tick("Static3dTree radiusKSearch");
      num_3d = static_3d_tree.radiusSearch(point_to_search, 0.4f, indices);
      time_counter::tock("Static3dTree radiusKSearch");

      time_counter::tick("KdTreeFLANN radiusKSearch");
      pcl_num_3d = pcl_kdtree.radiusSearch(pcl_point_to_search, 0.4f,
                                           pcl_indices, pcl_squared_distances);
      time_counter::tock("KdTreeFLANN radiusKSearch");
    }
  }
  time_counter::output("Static3dTree buildtree");
  time_counter::output("Static3dTree nearestKSearch");
  time_counter::output("Static3dTree radiusKSearch");
  time_counter::output("KdTreeFLANN buildtree");
  time_counter::output("KdTreeFLANN nearestKSearch");
  time_counter::output("KdTreeFLANN radiusKSearch");
  return 0;
}