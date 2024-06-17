#ifndef __MOTOR_MANAGER__
#define __MOTOR_MANAGER__


#include <array>
#include <utility>
#include <string>
#include <chrono>
#include <memory>

// Test Checks
#include <cassert>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

// Implemented
#include "Motor.hxx"

class MinimalMotor : public rclcpp::Node
{
public:
  MinimalMotor(std::string);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string id_;

  std::vector<double> initial_pose_;
  std::vector<double> initial_orientation_;
};




#endif