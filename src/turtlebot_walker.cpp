/**
 * @file turtlebot_walker.cpp
 * @author Uthappa Madettira (uthu@umd.edu)
 * @brief Contains the implementation of the Walker and RobotState class
 * @version 0.1
 * @date 2024-11-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "walker/turtlebot_walker.hpp"

// Walker class definition
Walker::Walker() : Node("walker") {
  // Publisher to publish Twist messages to cmd_vel topic
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Subscription to subscribe to LaserScan messages from scan topic
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Walker::scanCallback, this, std::placeholders::_1));
  // Initialize the state of the robot to MovingForward
  state_ = std::make_shared<MovingForward>();
  current_state_ = "MovingForward";
  previous_state_ = "MovingForward";
  rotating_clockwise_ = false;
  RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.c_str());
}

// Function to process LaserScan messages
void Walker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Define the forward range of the robot
  const double forward_min_angle = -M_PI / 5;
  const double forward_max_angle = M_PI / 5;

  // Initialize the minimum distance to infinity
  float min_distance = std::numeric_limits<float>::infinity();

  // Iterate through the ranges of the LaserScan message
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    if (angle >= forward_min_angle && angle <= forward_max_angle) {
      float range = msg->ranges[i];
      if (std::isfinite(range) && range < min_distance) {
        min_distance = range;
      }
    }
  }

  state_->execute(*this, min_distance);
}

// Function to change the state of the robot
void Walker::changeState(std::shared_ptr<RobotState> new_state) {
  previous_state_ = current_state_;
  current_state_ = typeid(new_state).name();
  state_ = new_state;
  RCLCPP_INFO(this->get_logger(), "State changed from %s to %s",
              previous_state_.c_str(), current_state_.c_str());
}

// Forward state
void MovingForward::execute(Walker &context, float min_distance) {
  if (min_distance < 0.8) {
    if (context.rotating_clockwise_) {
      context.changeState(std::make_shared<RotatingCounterClockwise>());
      context.rotating_clockwise_ = false;
    } else {
      context.changeState(std::make_shared<RotatingClockwise>());
      context.rotating_clockwise_ = true;
    }
  } else {
    auto msg = geometry_msgs::msg::Twist();
    // Set the linear velocity
    msg.linear.x = 0.2;
    context.publisher_->publish(msg);
  }
}

// Clockwise rotation state
void RotatingClockwise::execute(Walker &context, float min_distance) {
  if (min_distance > 0.8) {
    context.changeState(std::make_shared<MovingForward>());
  } else {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.4;
    context.publisher_->publish(msg);
  }
}

// Counter clockwise rotation state
void RotatingCounterClockwise::execute(Walker &context, float min_distance) {
  if (min_distance > 0.8) {
    context.changeState(std::make_shared<MovingForward>());
  } else {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = -0.4;
    context.publisher_->publish(msg);
  }
}

// Main function to run the walker node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}
