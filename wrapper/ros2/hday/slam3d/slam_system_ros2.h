#ifndef HDAY_SLAM3D_SLAM_SYSTEM_ROS2_H_
#define HDAY_SLAM3D_SLAM_SYSTEM_ROS2_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "hday/slam3d/slam_system/slam_system.h"

using ImuMsg = sensor_msgs::msg::Imu;

namespace hday {
namespace slam3d {

class SlamSystemRos2 : public rclcpp::Node {
 public:
  explicit SlamSystemRos2(const std::string& node_name) : Node(node_name) {
    // TODO(@chkim): load parameters from yaml file.
    hday::slam3d::slam_system::Parameters params;
    slam_ = std::make_unique<hday::slam3d::slam_system::SlamSystem>(params);

    InitializeSubscribers();

    RCLCPP_INFO(this->get_logger(), "SlamSystemRos2 node initialized");
  }

  ~SlamSystemRos2() {
    RCLCPP_INFO(this->get_logger(), "SlamSystemRos2 node destroyed");
  }

 private:
  void InitializeSubscribers() {
    imu_sub_ = this->create_subscription<ImuMsg>(
        "imu/data", rclcpp::QoS(10),
        std::bind(&SlamSystemRos2::ImuCallback, this, std::placeholders::_1));
  }

  void ImuCallback(const ImuMsg::SharedPtr msg) {
    hday::slam3d::ImuData imu_data;
    imu_data.timestamp.sec = msg->header.stamp.sec;
    imu_data.timestamp.nanosec = msg->header.stamp.nanosec;
    imu_data.angular_velocity.x() = msg->angular_velocity.x;
    imu_data.angular_velocity.y() = msg->angular_velocity.y;
    imu_data.angular_velocity.z() = msg->angular_velocity.z;
    imu_data.linear_acceleration.x() = msg->linear_acceleration.x;
    imu_data.linear_acceleration.y() = msg->linear_acceleration.y;
    imu_data.linear_acceleration.z() = msg->linear_acceleration.z;
    imu_data.orientation.w() = msg->orientation.w;
    imu_data.orientation.x() = msg->orientation.x;
    imu_data.orientation.y() = msg->orientation.y;
    imu_data.orientation.z() = msg->orientation.z;

    if (slam_ != nullptr) slam_->SetImuData(imu_data);
  }

  std::unique_ptr<hday::slam3d::slam_system::SlamSystem> slam_{nullptr};

  // Publishers

  // Subscribers
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
};

}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_SLAM_SYSTEM_ROS2_H_
