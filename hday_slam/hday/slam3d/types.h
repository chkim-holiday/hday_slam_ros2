#ifndef HDAY_SLAM3D_TYPES_H_
#define HDAY_SLAM3D_TYPES_H_

#include "Eigen/Dense"

namespace hday {
namespace slam3d {

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Mat2x2f = Eigen::Matrix2f;
using Mat3x3f = Eigen::Matrix3f;

using Vec3d = Eigen::Vector3d;
using Mat3x3d = Eigen::Matrix<double, 3, 3>;

using Pose = Eigen::Isometry3d;
using Quaternion = Eigen::Quaterniond;

struct Timestamp {
  uint32_t sec{0};
  uint32_t nanosec{0};
  double ToDouble() const { return static_cast<double>(sec) + nanosec * 1e-9; }
  bool operator<(const Timestamp& rhs) const {
    return (sec < rhs.sec) || (sec == rhs.sec && nanosec < rhs.nanosec);
  }
  bool operator<=(const Timestamp& rhs) const {
    return (sec < rhs.sec) || (sec == rhs.sec && nanosec <= rhs.nanosec);
  }
  bool operator==(const Timestamp& rhs) const {
    return sec == rhs.sec && nanosec == rhs.nanosec;
  }
};

struct ImuData {
  Timestamp timestamp;
  Vec3d linear_acceleration{Vec3d::Zero()};
  Vec3d angular_velocity{Vec3d::Zero()};
  Quaternion orientation{Quaternion::Identity()};  // Optional field. IMU does
                                                   // not always provide it.
};

struct OdometryData {
  Timestamp timestamp;
  Vec3d position{Vec3d::Zero()};
  Quaternion orientation{Quaternion::Identity()};
  Vec3d linear_velocity{Vec3d::Zero()};
  Vec3d angular_velocity{Vec3d::Zero()};
};

struct PointCloud {
  Timestamp timestamp;
  std::string sensor_id{""};
  std::vector<Vec3f> points;
};

struct AggregatedPointCloud {
  Timestamp timestamp;
  std::unordered_map<std::string, PointCloud> point_clouds;
};

}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_TYPES_H_
