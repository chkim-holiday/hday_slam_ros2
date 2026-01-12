#include "hday/slam3d/slam_system/slam_system.h"
#include "hday/slam3d/types.h"

namespace hday {
namespace slam3d {
namespace slam_system {

SlamSystem::SlamSystem(const Parameters& params) : params_(params) {}

void SlamSystem::SetImuData(const ImuData& imu_data) {
  if (point_cloud_aggregator_ != nullptr)
    point_cloud_aggregator_->SetImuData(imu_data);
}

}  // namespace slam_system
}  // namespace slam3d
}  // namespace hday
