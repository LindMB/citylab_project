#include "robot_patrol/go_to_pose_action.h"
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/server.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

GoToPose::GoToPose(const rclcpp::NodeOptions &options)
    : Node("go_to_pose_action_server_node", options) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot_1/odom", qos, std::bind(&GoToPose::odom_callback_, this, _1));

  auto timer_period = std::chrono::milliseconds(100); // 10Hz = 0.1s = 100ms
  this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/fastbot_1/cmd_vel", 10);
  this->cmd_vel_pub_timer_ = this->create_wall_timer(
      timer_period, std::bind(&GoToPose::cmd_vel_pub_timer_clbk_, this));

  this->goal_accepted_ = false;
  this->goal_reached_ = false;
  this->go_to_pose_action_server_ =
      rclcpp_action::create_server<GoToPoseAction>(
          this, "/go_to_pose", std::bind(&GoToPose::handle_goal_, this, _1, _2),
          std::bind(&GoToPose::handle_cancel_, this, _1),
          std::bind(&GoToPose::handle_accepted_, this, _1));
}

void GoToPose::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Retrieve robot position from msg
  this->current_pos_.x = msg->pose.pose.position.x;
  this->current_pos_.y = msg->pose.pose.position.y;

  // Retrieve robot orientation from msg
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Convert quaternion to angle (roll, pitch, yaw)
  // with yaw -> orientation around the z-axis
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  this->current_pos_.theta = yaw;
}

void GoToPose::cmd_vel_pub_timer_clbk_() {

  // If no goal has been set...
  if (!this->goal_accepted_) {
    return; // go to the next iteration...
  }

  // Compute the difference between desired_pos_ and current_pos_
  double dx = this->desired_pos_.x - this->current_pos_.x;
  double dy = this->desired_pos_.y - this->current_pos_.y;

  double distance_from_goal = std::sqrt(dx * dx + dy * dy);

  // Compute the angle (in [-pi, pi]) of the vector (dx, dy)
  // (ie between the robot and its goal)
  double target_angle = std::atan2(dy, dx);

  // Get the angle the robot has to rotate to be in front of the goal
  double angle_error = target_angle - this->current_pos_.theta;

  // Normalize the angle of how much the robot has to rotate
  // to be in front of the goal between [-pi, pi]
  angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

  auto goal_msg = geometry_msgs::msg::Twist();
  goal_msg.linear.x = 0.2;

  // If the distance between the goal is < 10cm ...
  if (distance_from_goal < 0.10) {

    goal_msg.linear.x = 0.0; // stop moving forward

    // Compute the difference of orientation
    // between desired_pos_ and current_pos_
    double delta_theta = this->desired_pos_.theta - this->current_pos_.theta;

    // Normalize the angle of how much the robot has to rotate to have
    // the same orientation as the goal in front of the goal between [-pi, pi]
    delta_theta = std::atan2(std::sin(delta_theta), std::cos(delta_theta));

    // If the robot has the same orientation as the goal...
    if (std::abs(delta_theta) < 0.05) { // threshold = 0.05

      goal_msg.angular.z = 0.0; // stop rotating

      this->goal_reached_ = true;
      this->goal_accepted_ = false;
    }
    // if the robot doesn't have the same orientation yet...
    else {
      if (delta_theta > 0.05) {
        goal_msg.angular.z = 0.5; // turn left
      } else if (delta_theta < -0.05) {
        goal_msg.angular.z = -0.5; // turn right
      } else {
        goal_msg.angular.z = 0.0; // stop rotating
      }
    }

  }
  // If the robot is not at the desired position yet...
  else {
    // if the goal is on the right side of the robot (-30 deg -> -180 deg)
    if (angle_error < -(M_PI / 6)) {
      goal_msg.angular.z = -0.5; // turn right
    }
    // if the goal is on the left side of the robot (+30 deg -> +180 deg)
    else if (angle_error >= (M_PI / 6)) {
      goal_msg.angular.z = 0.5; // turn left
    }
    // if the goal is in front of the robot (-30 deg -> +30 deg)
    else {
      goal_msg.angular.z = 0.0; // move forward
    }
  }

  this->cmd_vel_pub_->publish(goal_msg);
};

rclcpp_action::GoalResponse
GoToPose::handle_goal_(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const GoToPoseAction::Goal> goal) {

  (void)uuid;

  // Set the desired position to reach
  this->desired_pos_.x = goal->goal_pos.x;
  this->desired_pos_.y = goal->goal_pos.y;
  this->desired_pos_.theta = goal->goal_pos.theta;

  RCLCPP_INFO(this->get_logger(),
              "Received goal request with x: %.2f, y: %.2f, theta: %.2f",
              this->desired_pos_.x, this->desired_pos_.y,
              this->desired_pos_.theta);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
GoToPose::handle_cancel_(std::shared_ptr<GoalHandleGoToPose> goal_handle) {

  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPose::handle_accepted_(
    std::shared_ptr<GoalHandleGoToPose> goal_handle) {

  (void)goal_handle;

  this->goal_accepted_ = true;
  this->goal_reached_ = false;

  // Since handle_accepted_() is a callback function, if it takes too long
  // it will block the executor (=> it will block other callbacks, actions, etc)
  // This is why a new thread is created to run the real work function
  // execute() independently (ie .detach() )
  std::thread{std::bind(&GoToPose::execute_, this, _1), goal_handle}.detach();
}

void GoToPose::execute_(std::shared_ptr<GoalHandleGoToPose> goal_handle) {

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GoToPoseAction::Feedback>();
  auto result = std::make_shared<GoToPoseAction::Result>();

  rclcpp::Rate rate(1); // 1 msg per sec

  while (rclcpp::ok()) {

    // If the goal is canceled...
    if (goal_handle->is_canceling()) {

      RCLCPP_INFO(this->get_logger(), "Goal canceled");

      this->goal_accepted_ = false;
      this->goal_reached_ = false;

      // Stop the robot
      auto stop_msg = geometry_msgs::msg::Twist();
      this->cmd_vel_pub_->publish(stop_msg);

      // Indicate into the result that the goal has been canceled
      result->status = false;
      goal_handle->canceled(result);

      return;
    }

    if (this->goal_reached_) {

      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      result->status = true;
      goal_handle->succeed(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Navigating towards the desired position...");

    feedback->current_pos.x = this->current_pos_.x;
    feedback->current_pos.y = this->current_pos_.y;
    feedback->current_pos.theta = this->current_pos_.theta;

    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}