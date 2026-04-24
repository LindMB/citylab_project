#include "robot_patrol/patrol.h"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <math.h>

Patrol::Patrol(const std::string &node_name)
    : Node(node_name), node_name_(node_name) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/fastbot_1/scan", qos,
      std::bind(&Patrol::laserscan_callback_, this, std::placeholders::_1));
}

void Patrol::laserscan_callback_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // RCLCPP_INFO(this->get_logger(), "angle_min: %.2f | angle_max: %.2f |
  // angle_increment = %.2f", msg->angle_min, msg->angle_max,
  // msg->angle_increment);

  identify_safest_direction_to_move_next(msg);
}

void Patrol::identify_safest_direction_to_move_next(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  int index_min = (-M_PI_2 - msg->angle_min) / msg->angle_increment;
  int index_max = (-M_PI_2 - msg->angle_min) / msg->angle_increment;
  index_min = std::min(0, index_min);
  index_max = std::max((int)msg->ranges.size(), index_max);

  for (int i = index_min; i <= index_max; i++) {

    // If the ray length is different from inf, -inf and NAN AND is < 35cm
    if (std::isfinite(msg->ranges[i] && msg->ranges[i] < 0.35)) {

      RCLCPP_INFO(this->get_logger(), "Obstacle detected !");

      // Init variables for the longest ray research
      float max_length = -std::numeric_limits<float>::infinity();
      int max_length_index = -1;
      float length = 0.0;

      // Look for the longest ray in ranges between index_min and index_max
      // (included)
      for (int i = index_min; i <= index_max; i++) {

        length = msg->ranges[i];

        // If length is != from -inf, inf and NaN and is longer than the
        // previous max_length than...
        if (std::isfinite(length) && length > max_length) {
          max_length = length;
          max_length_index = i;
        }
      }

      // Angle regarding the ray position
      this->direction_ =
          msg->angle_min + (max_length_index * msg->angle_increment); // in rads

      RCLCPP_INFO(this->get_logger(),
                  "max_length: %.2f | max_length_index: %d | direction_ = %.2f",
                  max_length, max_length_index, this->direction_);
      break;
    } else {
      // Do nothing
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto robot_patrol_node = std::make_shared<Patrol>("robot_patrol_node");

  rclcpp::spin(robot_patrol_node);

  rclcpp::shutdown();
  return 0;
}