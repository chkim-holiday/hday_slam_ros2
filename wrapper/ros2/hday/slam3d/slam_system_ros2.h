#ifndef HDAY_SLAM3D_SLAM_SYSTEM_ROS2_H_
#define HDAY_SLAM3D_SLAM_SYSTEM_ROS2_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "hday/slam3d/slam_system.h"

namespace hday {
namespace slam3d {

class SlamSystemRos2 : public rclcpp::Node {
 public:
  explicit SlamSystemRos2(const std::string& node_name) : Node(node_name) {
    // TODO(@chkim): load parameters from yaml file.
    hday::slam3d::Parameters params;
    slam_ = std::make_unique<hday::slam3d::SlamSystem>(params);
    RCLCPP_INFO(this->get_logger(), "SlamSystemRos2 node initialized");
  }

  ~SlamSystemRos2() {
    RCLCPP_INFO(this->get_logger(), "SlamSystemRos2 node destroyed");
  }

 private:
  std::unique_ptr<hday::slam3d::SlamSystem> slam_{nullptr};
};

}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_SLAM_SYSTEM_ROS2_H_
