#include "robot_patrol/direction_service.h"
#include <memory>
#include <string>

DirectionService::DirectionService() : Node("direction_service") {

  std::string service_name = "/direction_service";
  this->direction_service_ =
      this->create_service<robot_patrol::srv::GetDirection>(
          service_name,
          std::bind(&DirectionService::direction_service_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
              service_name.c_str());
}

void DirectionService::direction_service_callback(
    const std::shared_ptr<GetDirection::Request> request,
    const std::shared_ptr<GetDirection::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Service Server Callback...");
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto direction_service_node = std::make_shared<DirectionService>();

  rclcpp::spin(direction_service_node);

  rclcpp::shutdown();
  return 0;
}