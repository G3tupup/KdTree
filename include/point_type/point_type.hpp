#ifndef POINT_TYPE_HPP_
#define POINT_TYPE_HPP_

#include <Eigen/Dense>
#include <iostream>

#define POINT3D                                                  \
  union EIGEN_ALIGN16 {                                          \
    float data[3];                                               \
    struct {                                                     \
      float x;                                                   \
      float y;                                                   \
      float z;                                                   \
    };                                                           \
  };                                                             \
  inline Eigen::Map<Eigen::Vector3f> point() {                   \
    return (Eigen::Vector3f::Map(data));                         \
  }                                                              \
  inline const Eigen::Map<const Eigen::Vector3f> point() const { \
    return (Eigen::Vector3f::Map(data));                         \
  }

namespace point_type {
//三维点
struct EIGEN_ALIGN16 Point3f {
  POINT3D;  //点坐标

  Point3f() = default;
  explicit Point3f(const float x_, const float y_, const float z_)
      : x(x_), y(y_), z(z_) {}
  explicit Point3f(const Eigen::Vector3f& point) { this->point() = point; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator<<(std::ostream& os, const Point3f& p) {
  os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return os;
}
}  // namespace point_type

#endif