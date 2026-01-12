#include "hday/slam3d/point_cloud_aggregator/point_cloud_aggregator.h"

#include <iostream>

namespace hday {
namespace slam3d {
namespace point_cloud_aggregator {

PointCloudAggregator::PointCloudAggregator(const Parameters& params)
    : params_(params) {}

void PointCloudAggregator::SetImuData(const ImuData& imu_data) {
  if (!imu_data_queue_.empty() &&
      imu_data.timestamp <= imu_data_queue_.front().timestamp) {
    std::cerr << "Imu timestamp is older than the latest data." << std::endl;
  }
  imu_data_queue_.push_back(imu_data);

  constexpr size_t kMaxQueueSize = 10000;
  if (imu_data_queue_.size() > kMaxQueueSize) imu_data_queue_.pop_front();
}

void PointCloudAggregator::SetOdometryData(const OdometryData& odometry_data) {
  if (!odometry_data_queue_.empty() &&
      odometry_data.timestamp <= odometry_data_queue_.front().timestamp) {
    std::cerr << "Odometry timestamp is older than the latest data."
              << std::endl;
  }
  odometry_data_queue_.push_back(odometry_data);

  constexpr size_t kMaxQueueSize = 10000;
  if (odometry_data_queue_.size() > kMaxQueueSize)
    odometry_data_queue_.pop_front();
}

}  // namespace point_cloud_aggregator
}  // namespace slam3d
}  // namespace hday
