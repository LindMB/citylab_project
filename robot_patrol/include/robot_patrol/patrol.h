#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

class Patrol : public rclcpp::Node {

public:
  Patrol(const std::string &node_name);

  ~Patrol() = default;

private:
  std::string node_name_;
  float direction_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;

  void laserscan_callback_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void identify_safest_direction_to_move_next(
      const sensor_msgs::msg::LaserScan::SharedPtr msg);
};