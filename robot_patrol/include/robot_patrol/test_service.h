#pragma once

#include "rclcpp/client.hpp"
#include "rclcpp/subscription.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class TestService : public rclcpp::Node {

public:
  TestService();

  ~TestService() = default;

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr
      direction_service_client_;

  void laserscan_callback_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void handle_service_response_(
      const rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
          result_future);
};