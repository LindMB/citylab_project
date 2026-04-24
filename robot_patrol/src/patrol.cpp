#include "robot_patrol/patrol.h"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <math.h>
#include <memory>

Patrol::Patrol(const std::string &node_name)
    : Node(node_name), node_name_(node_name) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/fastbot_1/scan", qos,
      std::bind(&Patrol::laserscan_callback_, this, std::placeholders::_1));

  this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/fastbot_1/cmd_vel", 10);

  auto timer_period = std::chrono::milliseconds(100); // 10Hz = 0.1s = 100ms

  this->cmd_vel_pub_timer_ = this->create_wall_timer(
      timer_period, std::bind(&Patrol::cmd_vel_pub_timer_clbk_, this));
}

void Patrol::laserscan_callback_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  /*RCLCPP_INFO(this->get_logger(),
              "angle_min: %.2f | angle_max: %.2f | angle_increment = %.2f",
              msg->angle_min, msg->angle_max, msg->angle_increment);*/

  this->obstacle_detected_ = is_obstacle_detected_(msg);

  if (this->obstacle_detected_) {
    identify_safest_direction_to_move_next(msg);
  } else {
    // Do nothing
  }
}

void Patrol::identify_safest_direction_to_move_next(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  int index_min = (-M_PI_2 - msg->angle_min) / msg->angle_increment;
  int index_max = (M_PI_2 - msg->angle_min) / msg->angle_increment;
  index_min = std::max(0, index_min);
  index_max = std::min((int)msg->ranges.size() - 1, index_max);

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
}

void Patrol::move_robot_forward_() {

  auto move_forward_msg = geometry_msgs::msg::Twist();
  move_forward_msg.linear.x = 0.1;
  move_forward_msg.angular.z = 0.0;
  this->cmd_vel_pub_->publish(move_forward_msg);
}

void Patrol::stop_robot() {

  auto stop_msg = geometry_msgs::msg::Twist(); // All fields default to zero
  this->cmd_vel_pub_->publish(stop_msg);
  RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
}

bool Patrol::is_obstacle_detected_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  int index_min = (-(M_PI / 12) - msg->angle_min) / msg->angle_increment;
  int index_max = ((M_PI / 12) - msg->angle_min) / msg->angle_increment;
  index_min = std::max(0, index_min);
  index_max = std::min((int)msg->ranges.size() - 1, index_max);

  for (int i = index_min; i <= index_max; i++) {

    // If the ray length is different from inf, -inf and NAN AND is < 35cm
    if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < 0.35) {

      RCLCPP_INFO(this->get_logger(), "Obstacle detected at %.2f m ! ",
                  msg->ranges[i]);
      return true;

    } else {
      // Do nothing
    }
  }

  return false;
}

void Patrol::cmd_vel_pub_timer_clbk_() {

  if (this->obstacle_detected_) {
    // Rotate to avoid obstacle
    auto avoid_msg = geometry_msgs::msg::Twist();
    avoid_msg.linear.x = 0.1;
    avoid_msg.angular.z = this->direction_ / 2;
    this->cmd_vel_pub_->publish(avoid_msg);

  } else {
    RCLCPP_INFO(this->get_logger(), "No obstacle in front of me.");
    move_robot_forward_();
  }
}

std::shared_ptr<Patrol> robot_patrol_node;

void signal_handler(int signum) {

  robot_patrol_node->stop_robot();
  rclcpp::shutdown();
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  robot_patrol_node = std::make_shared<Patrol>("robot_patrol_node");

  // Register the signal handler for CTRL+C
  std::signal(SIGINT, signal_handler);

  rclcpp::spin(robot_patrol_node);
  return 0;
}