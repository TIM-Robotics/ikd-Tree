// Minimal stand-in for <pcl/point_types.h> so ikd_Tree can be built and tested without a
// full PCL/ROS install. The tree only ever reads x/y/z from a point; the extra fields exist
// solely to keep the three explicit template instantiations (PointXYZ/PointXYZI/
// PointXYZINormal) compiling. Eigen::aligned_allocator is aliased to std::allocator since
// the tree only needs it as the PointVector allocator type.
#pragma once
#include <memory>

namespace Eigen {
template <typename T>
using aligned_allocator = std::allocator<T>;
}

namespace pcl {

struct PointXYZ {
  float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct PointXYZI {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float intensity = 0.0f;
};

struct PointXYZINormal {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float normal_x = 0.0f, normal_y = 0.0f, normal_z = 0.0f;
  float curvature = 0.0f;
};

}  // namespace pcl
