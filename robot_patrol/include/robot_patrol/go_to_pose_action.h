#pragma once

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::placeholders;

class GoToPose : public rclcpp::Node {

public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~GoToPose() = default;

private:
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  bool goal_reached_;
  bool goal_accepted_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr cmd_vel_pub_timer_;

  rclcpp_action::Server<GoToPoseAction>::SharedPtr go_to_pose_action_server_;

  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmd_vel_pub_timer_clbk_();

  // Action Server Callback functions
  rclcpp_action::GoalResponse
  handle_goal_(const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const GoToPoseAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel_(std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void handle_accepted_(std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void execute_(std::shared_ptr<GoalHandleGoToPose> goal_handle);
};