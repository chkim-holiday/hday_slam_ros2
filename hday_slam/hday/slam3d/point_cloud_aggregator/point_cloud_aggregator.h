#ifndef HDAY_SLAM3D_POINT_CLOUD_AGGREGATOR_POINT_CLOUD_AGGREGATOR_H_
#define HDAY_SLAM3D_POINT_CLOUD_AGGREGATOR_POINT_CLOUD_AGGREGATOR_H_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "hday/slam3d/types.h"

namespace hday {
namespace slam3d {
namespace point_cloud_aggregator {

struct Parameters {};

class PointCloudAggregator {
 public:
  explicit PointCloudAggregator(const Parameters& params);

  /// @brief Set IMU data for point cloud aggregation.
  /// @param imu_data IMU data to be set.
  void SetImuData(const ImuData& imu_data);

  /// @brief Set odometry data for point cloud aggregation.
  /// @param odometry_data Odometry data to be set.
  void SetOdometryData(const OdometryData& odometry_data);

 private:
  Parameters params_;

  std::deque<ImuData> imu_data_queue_;
  std::deque<OdometryData> odometry_data_queue_;

  std::unordered_map<std::string, Pose> sensor_extrinsic_pose_map_;
  std::unordered_map<std::string, std::deque<PointCloud>>
      point_cloud_queue_map_;
};

}  // namespace point_cloud_aggregator
}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_POINT_CLOUD_AGGREGATOR_POINT_CLOUD_AGGREGATOR_H_