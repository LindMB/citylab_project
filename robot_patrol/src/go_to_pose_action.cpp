#include "robot_patrol/go_to_pose_action.h"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/server.hpp"
#include <chrono>
#include <functional>
#include <memory>

GoToPose::GoToPose(const rclcpp::NodeOptions &options)
    : Node("go_to_pose_action_server_node", options) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot_1/odom", qos, std::bind(&GoToPose::odom_callback_, this, _1));

  auto timer_period = std::chrono::milliseconds(100); // 10Hz = 0.1s = 100ms
  this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/fastbot_1/cmd_vel", 10);

  this->go_to_pose_action_server_ =
      rclcpp_action::create_server<GoToPoseAction>(
          this, "/go_to_pose", std::bind(&GoToPose::handle_goal_, this, _1, _2),
          std::bind(&GoToPose::handle_cancel_, this, _1),
          std::bind(&GoToPose::handle_accepted_, this, _1));
}

void GoToPose::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {}

void GoToPose::cmd_vel_pub_timer_clbk_(){};

rclcpp_action::GoalResponse
GoToPose::handle_goal_(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const GoToPoseAction::Goal> goal) {

  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse
GoToPose::handle_cancel_(std::shared_ptr<GoalHandleGoToPose> goal_handle) {

  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
};

void GoToPose::handle_accepted_(
    std::shared_ptr<GoalHandleGoToPose> goal_handle) {

  (void)goal_handle;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}