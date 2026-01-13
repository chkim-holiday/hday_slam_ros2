#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "point_cloud_aggregator_ros2.h"

#define NODE_NAME "point_cloud_aggregator_node"

std::shared_ptr<rclcpp::Node> node = nullptr;
void HandleSignal(int signum) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Interrupt signal (%d) received. Shutting down gracefully...",
              signum);
  if (node) rclcpp::shutdown();
}

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);   // Ctrl+C
  std::signal(SIGTERM, HandleSignal);  // kill command

  try {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node = std::make_shared<PointCloudAggregatorRos2>(NODE_NAME, options);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Exception during node execution: %s", e.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Unknown exception during node execution");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node shutdown complete");
  rclcpp::shutdown();
  node.reset();
  return 0;
}
