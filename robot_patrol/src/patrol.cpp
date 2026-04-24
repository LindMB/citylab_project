#include "robot_patrol/patrol.h"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

Patrol::Patrol(const std::string &node_name)
    : Node(node_name), node_name_(node_name) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/fastbot_1/scan", qos,
      std::bind(&Patrol::laserscan_callback_, this, std::placeholders::_1));
}

void Patrol::laserscan_callback_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(),
              "angle_min: %.2f | angle_max: %.2f | angle_increment = %.2f",
              msg->angle_min, msg->angle_max, msg->angle_increment);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto robot_patrol_node = std::make_shared<Patrol>("robot_patrol_node");

  rclcpp::spin(robot_patrol_node);

  rclcpp::shutdown();
  return 0;
}