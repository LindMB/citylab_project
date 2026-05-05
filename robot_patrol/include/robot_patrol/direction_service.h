#pragma once

#include "robot_patrol/srv/get_direction.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

class DirectionService : public rclcpp::Node {

public:
  using GetDirection = robot_patrol::srv::GetDirection;

  DirectionService();
  ~DirectionService() = default;

private:
  rclcpp::Service<GetDirection>::SharedPtr direction_service_;

  void direction_service_callback(
      const std::shared_ptr<GetDirection::Request> request,
      const std::shared_ptr<GetDirection::Response> response);
};
