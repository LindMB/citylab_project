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

  std::vector<float> total_dist_sec_right;
  std::vector<float> total_dist_sec_front;
  std::vector<float> total_dist_sec_left;

  float angle_min = request->laser_data.angle_min;
  float angle = angle_min;
  float angle_increment = request->laser_data.angle_increment;

  for (int i = 0; i < (int)request->laser_data.ranges.size(); i++) {

    angle = angle_min + (i * angle_increment);

    // Normalize angle
    // from [0, 2pi] (lidar in real life) to [-pi, pi] (lidar in simulation)
    if (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    // If the ray length is equal to inf, -inf and NAN then...
    if (!std::isfinite(request->laser_data.ranges[i])) {
      continue; // Go to the next iteration directly
    }

    // For ray in the frontal section (ie [-pi/2, pi/2])
    if (angle >= -(M_PI / 2) && angle <= (M_PI / 2)) {

      //  right  : -90 deg -> -30 deg
      if (angle >= -(M_PI / 2) && angle < -(M_PI / 6)) {
        total_dist_sec_right.push_back(request->laser_data.ranges[i]);
      }
      // front   : -30 deg -> +30 deg
      else if (angle >= -(M_PI / 6) && angle < (M_PI / 6)) {
        total_dist_sec_front.push_back(request->laser_data.ranges[i]);
      }
      // left    : +30 deg -> +90 deg
      else if (angle >= (M_PI / 6) && angle <= (M_PI / 2)) {
        total_dist_sec_left.push_back(request->laser_data.ranges[i]);
      }
    }
  }

  // Compute the sum of elements in total_dist_sec_right vector
  float right_sum = 0.0;
  for (auto element : total_dist_sec_right) {
    right_sum += element;
  }

  // Compute the sum of elements in total_dist_sec_front vector
  float front_sum = 0.0;
  for (auto element : total_dist_sec_front) {
    front_sum += element;
  }

  // Compute the sum of elements in total_dist_sec_left vector
  float left_sum = 0.0;
  for (auto element : total_dist_sec_left) {
    left_sum += element;
  }

  // Choose the section towards which the robot must move to
  if (right_sum >= front_sum && right_sum >= left_sum) {
    response->direction = "right";

  } else if (left_sum >= right_sum && left_sum >= front_sum) {
    response->direction = "left";

  } else {
    response->direction = "forward";
  }
  RCLCPP_INFO(this->get_logger(), "Right: %.2f | Front: %.2f | Left: %.2f",
              right_sum, front_sum, left_sum);
  RCLCPP_INFO(this->get_logger(), "Go: %s", response->direction.c_str());
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto direction_service_node = std::make_shared<DirectionService>();

  rclcpp::spin(direction_service_node);

  rclcpp::shutdown();
  return 0;
}