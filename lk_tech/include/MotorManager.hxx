#ifndef __MOTOR_MANAGER__
#define __MOTOR_MANAGER__


#include <array>
#include <utility>
#include <string>
#include <chrono>
#include <memory>

#include <chrono>

// Test Checks
#include <cassert>

// ROS2
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


// Implemented
#include "Motor.hxx"

class MinimalMotor : public rclcpp::Node
{
public:
  MinimalMotor(std::string);
  ~MinimalMotor();

  auto warmUp() -> void;
  auto matchPort() -> void;

  auto callback(std_msgs::msg::Float32::SharedPtr) -> void;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint8_t id_;
  std::shared_ptr<Motor> motor_;
  std::string port_address_;
  bool motor_found_{};

  std::vector<double> initial_pose_;
  std::vector<double> initial_orientation_;
  std::vector<std::string> port_list_;




  // Publisher and Subscriber
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

  std_msgs::msg::Float32MultiArray feedback_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};




#endif