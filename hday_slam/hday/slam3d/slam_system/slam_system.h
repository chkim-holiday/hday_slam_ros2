#ifndef HDAY_SLAM3D_SLAM_SYSTEM_SLAM_SYSTEM_H_
#define HDAY_SLAM3D_SLAM_SYSTEM_SLAM_SYSTEM_H_

#include <deque>
#include <memory>

#include "hday/slam3d/point_cloud_aggregator/point_cloud_aggregator.h"
#include "hday/slam3d/types.h"

namespace hday {
namespace slam3d {
namespace slam_system {

struct Parameters {
  point_cloud_aggregator::Parameters point_cloud_aggregator_params;
};

class SlamSystem {
 public:
  explicit SlamSystem(const Parameters& params);

  void SetImuData(const ImuData& imu_data);

 private:
  Parameters params_;

  std::unique_ptr<point_cloud_aggregator::PointCloudAggregator>
      point_cloud_aggregator_{nullptr};
};

}  // namespace slam_system
}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_SLAM_SYSTEM_SLAM_SYSTEM_H_
