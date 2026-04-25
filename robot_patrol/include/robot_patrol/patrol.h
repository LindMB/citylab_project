#pragma once

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

class Patrol : public rclcpp::Node {

public:
  Patrol(const std::string &node_name);

  void stop_robot();

  ~Patrol() = default;

private:
  std::string node_name_;
  float direction_;
  bool obstacle_detected_ = false;
  bool first_odom_ = false;
  double previous_yam_;
  double accumulated_yaw_;
  bool lap_completed_ = false;
  bool turn_around_completed_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr cmd_vel_pub_timer_;

  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laserscan_callback_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool is_obstacle_detected_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void identify_safest_direction_to_move_next(
      const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void move_robot_forward_();
  void turn_robot_around_();
  void cmd_vel_pub_timer_clbk_();
};