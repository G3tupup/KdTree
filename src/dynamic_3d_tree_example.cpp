#include <iostream>

#include "../include/kdtree/dynamic_3d_tree.hpp"
#include "../include/point_cloud/point_cloud.hpp"
#include "../include/point_type/point_type.hpp"

int main() {
  kd_tree::Dynamic3dTree<point_cloud::PointCloud<point_type::Point3d>>
      dynamic_3d_tree;
  std::vector<point_type::Point3d> new_points;
  std::cout << "======================First Add========================"
            << std::endl;
  for (int i = 0; i < 5; i++) {
    new_points.push_back(point_type::Point3d(static_cast<double>(i), 0.0,
                                             static_cast<double>(i)));
  }
  dynamic_3d_tree.add(new_points);

  point_type::Point3d point_to_search(4.2, 0.0, 3.9);
  std::vector<size_t> indices;
  std::vector<double> squared_distances;

  std::cout << "=====================nearestKSearch======================"
            << std::endl;
  size_t num_3d = dynamic_3d_tree.nearestKSearch(point_to_search, 6, indices,
                                                 squared_distances);

  std::cout << "Find:" << num_3d << "points," << std::endl;
  for (size_t i = 0; i < num_3d; i++) {
    std::cout << "Point" << i << ": " << dynamic_3d_tree[indices[i]];
    std::cout << ", Distance: " << std::sqrt(squared_distances[i]) << std::endl;
  }
  std::cout << "======================radiusSearch======================="
            << std::endl;
  num_3d = dynamic_3d_tree.radiusSearch(point_to_search, 8.5, indices);
  std::cout << "Find:" << num_3d << "points," << std::endl;
  for (size_t i = 0; i < num_3d; i++) {
    std::cout << "Point" << i << ": " << dynamic_3d_tree[indices[i]]
              << std::endl;
  }

  std::cout << "======================Second Add========================="
            << std::endl;
  new_points.clear();
  for (size_t i = 5; i < 20; i++) {
    new_points.push_back(point_type::Point3d(static_cast<double>(i), 0.0,
                                             static_cast<double>(i)));
  }
  dynamic_3d_tree.add(new_points);

  std::cout << "=====================nearestKSearch======================"
            << std::endl;
  num_3d = dynamic_3d_tree.nearestKSearch(point_to_search, 6, indices,
                                          squared_distances);

  std::cout << "Find:" << num_3d << "points," << std::endl;
  for (size_t i = 0; i < num_3d; i++) {
    std::cout << "Point" << i << ": " << dynamic_3d_tree[indices[i]];
    std::cout << ", Distance: " << std::sqrt(squared_distances[i]) << std::endl;
  }
  std::cout << "======================radiusSearch======================="
            << std::endl;
  num_3d = dynamic_3d_tree.radiusSearch(point_to_search, 8.5, indices);
  std::cout << "Find:" << num_3d << "points," << std::endl;
  for (size_t i = 0; i < num_3d; i++) {
    std::cout << "Point" << i << ": " << dynamic_3d_tree[indices[i]]
              << std::endl;
  }

  return 0;
}