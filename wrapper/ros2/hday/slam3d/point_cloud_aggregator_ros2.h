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

struct SensorTopicInfo {
  std::vector<std::pair<std::string, std::string>> lidar_name_and_topic_names;
  std::pair<std::string, std::string> imu_name_and_topic_name;
};

class PointCloudAggregatorRos2 : public rclcpp::Node {
 public:
  explicit PointCloudAggregatorRos2(
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node(node_name, options) {
    // Initialize core
    hday::slam3d::point_cloud_aggregator::Parameters parameters;
    if (!LoadParameters(&parameters))
      throw std::runtime_error("Failed to load parameters.");
    point_cloud_aggregator_ = std::make_unique<
        hday::slam3d::point_cloud_aggregator::PointCloudAggregator>(parameters);

    // Initialize ROS2 subscribers
    SensorTopicInfo sensor_topic_info;
    if (!LoadSensorTopicInfo(&sensor_topic_info))
      throw std::runtime_error("Failed to load sensor topic informations.");
    InitializeSubscribers(sensor_topic_info);
  }

  ~PointCloudAggregatorRos2() {
    RCLCPP_INFO(this->get_logger(), "PointCloudAggregatorRos2 node destroyed");
  }

 private:
  bool LoadSensorTopicInfo(SensorTopicInfo* sensor_topic_info) {
    bool success = true;

    std::vector<std::string> lidar_names;
    std::vector<std::string> lidar_topic_names;
    success &= get_parameter_or("slam3d.sensors.lidar", lidar_names,
                                std::vector<std::string>{});
    std::cout << "Total (" << lidar_names.size() << ") lidars are found:\n";
    std::cout << "  sensor name:topic name\n";
    lidar_topic_names.resize(lidar_names.size());
    for (size_t i = 0; i < lidar_names.size(); ++i) {
      const std::string& lidar_name = lidar_names[i];
      std::string& lidar_topic_name = lidar_topic_names[i];
      success &= get_parameter_or("sensors.lidar." + lidar_name + ".topic_name",
                                  lidar_topic_name, std::string{});
      if (lidar_topic_name.empty())
        RCLCPP_ERROR(this->get_logger(),
                     "Lidar sensor '%s' has no topic_name parameter.",
                     lidar_name.c_str());
      std::cout << "  " << lidar_name << ":" << lidar_topic_name << std::endl;
    }

    std::string imu_name;
    std::string imu_topic_name;
    success &= get_parameter_or("slam3d.sensors.imu", imu_name, std::string{});
    success &= get_parameter_or("sensors.imu." + imu_name + ".topic_name",
                                imu_topic_name, std::string{});
    std::cout << "Loaded IMU: " << std::endl;
    std::cout << "  sensor name:topic name\n";
    std::cout << "  " << imu_name << ":" << imu_topic_name << std::endl;

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load sensor informations.");
      return false;
    }

    sensor_topic_info->lidar_name_and_topic_names.clear();
    sensor_topic_info->lidar_name_and_topic_names.reserve(lidar_names.size());
    for (size_t i = 0; i < lidar_names.size(); ++i) {
      sensor_topic_info->lidar_name_and_topic_names.emplace_back(
          lidar_names[i], lidar_topic_names[i]);
    }
    sensor_topic_info->imu_name_and_topic_name = {imu_name, imu_topic_name};

    return success;
  }

  bool LoadParameters(
      hday::slam3d::point_cloud_aggregator::Parameters* parameters) {
    // TODO(@chkim): Load parameters from ROS2 parameter server
    bool success = true;
    (void)success;
    (void)parameters;

    return success;
  }

  void InitializeSubscribers(const SensorTopicInfo& sensor_topic_info) {
    // Subscribe lidars
    const auto& lidar_name_and_topic_names =
        sensor_topic_info.lidar_name_and_topic_names;
    point_cloud2_subs_.clear();
    point_cloud2_subs_.reserve(lidar_name_and_topic_names.size());
    for (const auto& [sensor_name, topic_name] : lidar_name_and_topic_names) {
      point_cloud2_subs_.insert(
          {sensor_name,
           this->create_subscription<PointCloud2Msg>(
               topic_name, rclcpp::QoS(10),
               [this, sensor_name](const PointCloud2Msg::SharedPtr msg) {
                 this->PointCloud2Callback(sensor_name, msg);
               })});
    }

    // Subscribe IMU
    const auto& imu_name_and_topic_name =
        sensor_topic_info.imu_name_and_topic_name;
    imu_sub_ = this->create_subscription<ImuMsg>(
        imu_name_and_topic_name.second, rclcpp::QoS(10),
        std::bind(&PointCloudAggregatorRos2::ImuCallback, this,
                  std::placeholders::_1));
  }

  void PointCloud2Callback(const std::string& sensor_name,
                           const PointCloud2Msg::SharedPtr msg) {
    (void)sensor_name;
    (void)msg;
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
