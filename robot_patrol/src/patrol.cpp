#include "robot_patrol/patrol.h"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <math.h>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

Patrol::Patrol(const std::string &node_name)
    : Node(node_name), node_name_(node_name) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&Patrol::laserscan_callback_, this, std::placeholders::_1));

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      std::bind(&Patrol::odom_callback_, this, std::placeholders::_1));

  this->cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  auto timer_period = std::chrono::milliseconds(100); // 10Hz = 0.1s = 100ms

  this->cmd_vel_pub_timer_ = this->create_wall_timer(
      timer_period, std::bind(&Patrol::cmd_vel_pub_timer_clbk_, this));
}

void Patrol::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Retrieve robot position from msg
  this->current_pos_x_ = msg->pose.pose.position.x;
  this->current_pos_y_ = msg->pose.pose.position.y;

  // Retrieve robot orientation from msg
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Init the robot starting position
  if (!this->start_position_initialized_) {
    this->start_pos_x_ = this->current_pos_x_;
    this->start_pos_y_ = this->current_pos_y_;

    this->previous_pos_x_ = this->current_pos_x_;
    this->previous_pos_y_ = this->current_pos_y_;

    this->start_position_initialized_ = true;
  }

  // Convert quaternion to angle (roll, pitch, yaw)
  // with yaw -> orientation around the z-axis
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // For the first odom msg, just init previous_yam_
  if (first_odom_) {
    this->previous_yam_ = yaw;
    this->first_odom_ = false;
    return;
  }

  // Calculate the change in position between the current and previous positions
  double dx = this->current_pos_x_ - this->previous_pos_x_;
  double dy = this->current_pos_y_ - this->previous_pos_y_;

  // Calculate the total distance traveled since the robot is moving
  this->traveled_distance_ += std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO(this->get_logger(), "traveled_distance_ : %.2f",
              this->traveled_distance_);

  // Update the previous robot position
  this->previous_pos_x_ = this->current_pos_x_;
  this->previous_pos_y_ = this->current_pos_y_;

  // Calculate the variation angle around the z-axis
  double delta_yaw = yaw - this->previous_yam_;

  // Normalize the delta_yaw to the range [-pi, pi]
  // delta_yaw = std::fmod(delta_yaw + M_PI, 2 * M_PI) - M_PI;

  if (delta_yaw > M_PI) {
    delta_yaw -= 2.0 * M_PI;
  } else if (delta_yaw < -M_PI) {
    delta_yaw += 2.0 * M_PI;
  }

  // Calculate the total robot rotation done since the robot started moving

  // Ignore small noise-induced movements
  double filtered_delta = 0.0;

  if (std::abs(delta_yaw) > 0.02) { // Threshold for actual movements
    filtered_delta = delta_yaw;
  } else {
    filtered_delta = 0.0;
  }

  // If the robot is currently doing the 180 deg turn
  if (this->lap_completed_ && !this->turn_around_completed_) {

    this->accumulated_turn_yaw_ += std::abs(filtered_delta);

    RCLCPP_INFO(this->get_logger(), "accumulated_turn_yaw_ : %.2f",
                this->accumulated_turn_yaw_);
  }

  // Update previous_yaw for the next calculation
  this->previous_yam_ = yaw;

  // Calculate the change in position between the current and starting positions
  double dx_start = this->current_pos_x_ - this->start_pos_x_;
  double dy_start = this->current_pos_y_ - this->start_pos_y_;

  // Calculate the distance from where the robot is currently
  // to where its starting point is
  double distance_from_start =
      std::sqrt(dx_start * dx_start + dy_start * dy_start);

  RCLCPP_INFO(this->get_logger(), "distance_from_start : %.2f",
              distance_from_start);

  // If the distance between the robot and its starting point is less than 35cm
  // AND if the robot traveled more than 2.0 meters
  if (distance_from_start < 0.35 && this->traveled_distance_ > 2.0 &&
      !this->lap_completed_) {

    RCLCPP_INFO(this->get_logger(), "1 full lap completed !");

    this->lap_completed_ = true;

    // Prepare robot to turn around after a lap completed
    this->turn_around_completed_ = false;
    this->accumulated_turn_yaw_ = 0.0;
  }
}

void Patrol::laserscan_callback_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  /*RCLCPP_INFO(this->get_logger(),
              "angle_min: %.2f | angle_max: %.2f | angle_increment = %.2f",
              msg->angle_min, msg->angle_max, msg->angle_increment);*/

  if (is_obstacle_detected_(msg)) {
    identify_safest_direction_to_move_next(msg);
  } else {
    // Do nothing
  }
}

void Patrol::identify_safest_direction_to_move_next(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  double angle = msg->angle_min;

  // Init variables for the longest ray research
  float max_length = -std::numeric_limits<float>::infinity();
  int max_length_index = -1;
  float length = 0.0;

  for (int i = 0; i < (int)msg->ranges.size(); i++) {

    angle = msg->angle_min + (i * msg->angle_increment);

    // Normalize angle
    // from [0, 2pi] (lidar in real life) to [-pi, pi] (lidar in simulation)
    if (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    // 1) If an obstacle is present on the left, look for the longest ray on the
    // right side
    // 2) If an obstacle is present on the right, look for the longest
    // ray on the left side
    // 3) If an obstacle is present on the front (left and
    // right at true), look for the longest ray on both sides
    if (((this->obstacle_detected_on_the_left_) &&
         (angle < 0 && angle >= -M_PI_2)) ||
        ((this->obstacle_detected_on_the_right_) &&
         (angle <= M_PI_2 && angle >= 0))) {

      length = msg->ranges[i];

      // If length is != from -inf, inf and NaN and is longer than the
      // previous max_length than...
      if (std::isfinite(length) && length > max_length) {
        max_length = length;
        max_length_index = i;

        // Store the normalised angle (in [-pi, pi]) regarding the ray position
        this->direction_ = angle;
      }
    }
  }

  // If no valid ray is found...
  if (max_length_index == -1) {
    this->direction_ = -0.2; // turn right slowly
  }

  // RCLCPP_INFO(this->get_logger(),
  //"max_length: %.2f | max_length_index: %d | direction_ = %.2f",
  // max_length, max_length_index, this->direction_);
}

void Patrol::move_robot_forward_() {

  auto move_forward_msg = geometry_msgs::msg::Twist();
  move_forward_msg.linear.x = 0.1;
  move_forward_msg.angular.z = 0.0;
  this->cmd_vel_pub_->publish(move_forward_msg);
}

void Patrol::turn_robot_around_() {

  auto turn_around_msg = geometry_msgs::msg::Twist();

  // If the robot has not finish to turn around
  if (std::abs(this->accumulated_turn_yaw_) < M_PI) {

    RCLCPP_INFO(this->get_logger(), "I'm turning around...");
    turn_around_msg.angular.z = 0.2; // Continue to rotate

  } else {

    RCLCPP_INFO(this->get_logger(), "Turn around completed !");
    turn_around_msg.angular.z = 0.0; // Stop rotating

    this->turn_around_completed_ = true;

    // Prepare robot to detect if a new lap is completed
    this->lap_completed_ = false;

    // Prepare robot to calculate accumulated_turn_yaw_ for the full lap
    this->accumulated_turn_yaw_ = 0.0;

    // Reset all positions for the next lap
    this->start_pos_x_ = this->current_pos_x_;
    this->start_pos_y_ = this->current_pos_y_;

    this->previous_pos_x_ = this->current_pos_x_;
    this->previous_pos_y_ = this->current_pos_y_;

    // Reset the robot traveled distance for the next lap
    this->traveled_distance_ = 0.0;
  }

  this->cmd_vel_pub_->publish(turn_around_msg);
}

void Patrol::avoid_obstacle_() {

  auto avoid_msg = geometry_msgs::msg::Twist();
  avoid_msg.linear.x = 0.05;
  avoid_msg.angular.z = this->direction_ / 2;
  this->cmd_vel_pub_->publish(avoid_msg);
}

void Patrol::stop_robot() {

  auto stop_msg = geometry_msgs::msg::Twist(); // All fields default to zero
  this->cmd_vel_pub_->publish(stop_msg);
  RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
}

bool Patrol::is_obstacle_detected_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // Re init obstacle detection at each laserscan msg reading
  // to check where the obstacle is in comparison to the robot
  this->obstacle_detected_on_the_left_ = false;
  this->obstacle_detected_on_the_right_ = false;

  double angle;

  for (int i = 0; i < (int)msg->ranges.size(); i++) {

    angle = msg->angle_min + (i * msg->angle_increment);

    // Normalize angle
    // from [0, 2pi] (lidar in real life) to [-pi, pi] (lidar in simulation)
    if (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    // If the ray length is different from inf, -inf and NAN AND is < 35cm
    if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < 0.35) {

      // For a ray of the front-left section (between 30deg and 45deg) of the
      // lidar...
      if (angle >= M_PI / 12 && angle <= M_PI / 6) {
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected at %.2f m on the left ! ",
                    msg->ranges[i]);
        this->obstacle_detected_on_the_left_ = true;
      }

      // For a ray of the front-right section (between -30deg and -45deg) of the
      // lidar...
      else if (angle <= -M_PI / 12 && angle >= -M_PI / 6) {
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected at %.2f m on the right !",
                    msg->ranges[i]);
        this->obstacle_detected_on_the_right_ = true;
      }

      // For a ray of the very front section (between -10deg and
      // 10deg) of the lidar...
      else if (angle <= -(M_PI / 18) && angle >= -(M_PI / 10)) {
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected at %.2f m on the very front !",
                    msg->ranges[i]);
        // Both flags at true to allow to look for the safest direction
        // between [0, pi] and [-pi/2, 0]
        this->obstacle_detected_on_the_left_ = true;
        this->obstacle_detected_on_the_right_ = true;
      }
    }
  }

  // Return true if at least one of them is true
  return this->obstacle_detected_on_the_left_ ||
         this->obstacle_detected_on_the_right_;
}

void Patrol::cmd_vel_pub_timer_clbk_() {

  // When a lap has been completed, do a 180deg turn
  // and when turning around, ignore obstacle detection
  if (this->lap_completed_ && !(this->turn_around_completed_)) {

    turn_robot_around_();
  }
  // If obstacle detected
  else if (this->obstacle_detected_on_the_left_ ||
           this->obstacle_detected_on_the_right_) {

    avoid_obstacle_();

  } else {
    move_robot_forward_();
  }
}

std::shared_ptr<Patrol> robot_patrol_node;

void signal_handler(int /*signum*/) { // intentionally unused

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