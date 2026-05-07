#include "robot_patrol/test_service.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <future>
#include <memory>
#include <rclcpp/qos.hpp>
#include <string>

TestService::TestService() : Node("test_service") {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  this->laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&TestService::laserscan_callback_, this,
                std::placeholders::_1));

  const std::string service_name = "/direction_service";
  this->direction_service_client_ =
      this->create_client<robot_patrol::srv::GetDirection>(service_name);

  // Wait for the service to be available (checks every second)
  while (
      !direction_service_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Service %s not available, waiting again...",
                service_name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
              service_name.c_str());
}

void TestService::handle_service_response_(
    const rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
        result_future) {

  // Get the service server's response
  auto response = result_future.get();

  RCLCPP_INFO(this->get_logger(), "Service Response: direction = %s",
              response->direction.c_str());
}

void TestService::laserscan_callback_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
  // Fill request with msg
  request->laser_data = *msg;

  if (!direction_service_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "Failed to call service");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Service Request...");
  // Send the request asynchronously
  // and when a response is received, use the handle_service_response_ fct
  auto result_future = this->direction_service_client_->async_send_request(
      request, std::bind(&TestService::handle_service_response_, this,
                         std::placeholders::_1));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto direction_service_client_node = std::make_shared<TestService>();

  rclcpp::spin(direction_service_client_node);

  rclcpp::shutdown();
  return 0;
}