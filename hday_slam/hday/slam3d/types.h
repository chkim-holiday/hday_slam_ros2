#ifndef HDAY_SLAM3D_TYPES_H_
#define HDAY_SLAM3D_TYPES_H_

#include "Eigen/Dense"

namespace hday {
namespace slam3d {

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Mat2x2f = Eigen::Matrix2f;
using Mat3x3f = Eigen::Matrix3f;

using Pose = Eigen::Isometry3d;

struct ImuData {
  double timestamp{0.0};
  Vec3d linear_acceleration{Vec3d::Zero()};
  Vec3d angular_velocity{Vec3d::Zero()};
};

}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_TYPES_H_
