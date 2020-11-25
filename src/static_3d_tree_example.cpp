#include <iostream>

#include "../include/kdtree/static_3d_tree.hpp"
#include "../include/point_cloud/point_cloud.hpp"
#include "../include/point_type/point_type.hpp"

int main() {
  kd_tree::Static3dTree<point_cloud::PointCloud<point_type::Point3f>>
      static_3d_tree;
  for (int i = 0; i < 20; i++) {
    static_3d_tree.add(
        point_type::Point3f(static_cast<float>(i), 0.f, static_cast<float>(i)));
  }
  static_3d_tree.build();

  point_type::Point3f point_to_search(4.2f, 0.f, 3.9f);
  std::vector<size_t> indices;
  std::vector<float> squared_distances;

  std::cout << "=====================nearestKSearch======================"
            << std::endl;
  size_t num_3d = static_3d_tree.nearestKSearch(point_to_search, 6, indices,
                                                squared_distances);

  std::cout << "Find:" << num_3d << "points," << std::endl;
  for (size_t i = 0; i < num_3d; i++) {
    std::cout << "Point" << i << ": " << static_3d_tree[indices[i]];
    std::cout << ", Distance: " << std::sqrt(squared_distances[i]) << std::endl;
  }
  std::cout << "======================radiusSearch======================="
            << std::endl;
  num_3d = static_3d_tree.radiusSearch(point_to_search, 8.5f, indices);
  std::cout << "Find:" << num_3d << "points," << std::endl;
  for (size_t i = 0; i < num_3d; i++) {
    std::cout << "Point" << i << ": " << static_3d_tree[indices[i]]
              << std::endl;
  }

  return 0;
}