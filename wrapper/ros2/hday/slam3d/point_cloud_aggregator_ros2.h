#ifndef HDAY_SLAM3D_POINT_CLOUD_AGGREGATOR_ROS2_H_
#define HDAY_SLAM3D_POINT_CLOUD_AGGREGATOR_ROS2_H_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "hday/slam3d/point_cloud_aggregator/point_cloud_aggregator.h"
#include "hday/slam3d/types.h"

using ImuMsg = sensor_msgs::msg::Imu;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;

class PointCloudAggregatorRos2 : public rclcpp::Node {
 public:
  explicit PointCloudAggregatorRos2(const std::string& node_name)
      : Node(node_name) {
    // TODO(@chkim): load parameters from yaml file.
    hday::slam3d::point_cloud_aggregator::Parameters params;
    point_cloud_aggregator_ = std::make_unique<
        hday::slam3d::point_cloud_aggregator::PointCloudAggregator>(params);

    InitializeSubscribers();

    RCLCPP_INFO(this->get_logger(),
                "PointCloudAggregatorRos2 node initialized");
  }

  ~PointCloudAggregatorRos2() {
    RCLCPP_INFO(this->get_logger(), "PointCloudAggregatorRos2 node destroyed");
  }

 private:
  void InitializeSubscribers() {
    std::vector<std::string> sensor_names = {"lidar_front", "lidar_rear",
                                             "lidar_left", "lidar_right"};

    for (const auto& sensor_name : sensor_names) {
      point_cloud2_subs_[sensor_name] =
          this->create_subscription<PointCloud2Msg>(
              "/sensors/" + sensor_name + "/point_cloud", rclcpp::QoS(10),
              [this, sensor_name](const PointCloud2Msg::SharedPtr msg) {
                this->PointCloud2Callback(sensor_name, msg);
              });
    }

    imu_sub_ = this->create_subscription<ImuMsg>(
        "imu/data", rclcpp::QoS(10),
        std::bind(&PointCloudAggregatorRos2::ImuCallback, this,
                  std::placeholders::_1));
  }

  void PointCloud2Callback(const std::string& sensor_name,
                           const PointCloud2Msg::SharedPtr msg) {
    (void)sensor_name;
    if (point_cloud_aggregator_ != nullptr) {
      // TODO(@chkim): Convert PointCloud2Msg to PointCloud and call
      // AddPointCloud.
    }
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

    if (point_cloud_aggregator_ != nullptr)
      point_cloud_aggregator_->SetImuData(imu_data);
  }

  std::unique_ptr<hday::slam3d::point_cloud_aggregator::PointCloudAggregator>
      point_cloud_aggregator_{nullptr};

  // Subscribers
  std::unordered_map<std::string,
                     rclcpp::Subscription<PointCloud2Msg>::SharedPtr>
      point_cloud2_subs_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
};

#endif  // HDAY_SLAM3D_POINT_CLOUD_AGGREGATOR_ROS2_H_
