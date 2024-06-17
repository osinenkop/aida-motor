#include "MotorManager.hxx"

// SingleMotorManager::SingleMotorManager(std::string node_name): Node(node_name){
//     // this -> declare_parameter("front_left_id", 1);
//     // this -> declare_parameter("front_right_id", 2);
//     // this -> declare_parameter("rear_left_id", 3);
//     // this -> declare_parameter("rear_right_id", 4);
//     // this -> declare_parameter("front_rear_dist", 1);
//     // this -> declare_parameter("left_right_dist", 1);
//     this -> declare_parameter("id", 1);
//     this -> id_ = this->get_parameter("id").as_string();
//     this -> id_.pop_back();

//     // this -> port_id_pair_[0].second = this->get_parameter("front_left_id").as_int();
//     // this -> port_id_pair_[1].second = this->get_parameter("front_right_id").as_int();
//     // this -> port_id_pair_[2].second = this->get_parameter("rear_left_id").as_int();
//     // this -> port_id_pair_[3].second = this->get_parameter("rear_right_id").as_int();

//     // this -> front_rear_dist = static_cast<float>(this->get_parameter("front_rear_dist").as_double());
//     // this -> left_right_dist = static_cast<float>(this->get_parameter("left_right_dist").as_double());

//     // this -> port_list_ = this->get_parameter("port_list").as_string_array();
    
//     this -> queue_size_ = 1;
//     this -> fps_ = 90;

//     // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", node_name.c_str());
// }



MinimalMotor::MinimalMotor(std::string node_name): Node(node_name)
  {
    // std::string node_name{"Hey there"};

    this -> declare_parameter("id", rclcpp::PARAMETER_STRING);
    this -> id_ = this->get_parameter("id").as_string();
    this -> id_.pop_back();

    this -> declare_parameter("initial_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this -> initial_pose_ = this->get_parameter("initial_pose").as_double_array();

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", node_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Device ID: '%s'", this -> id_.c_str()); 
    RCLCPP_INFO(this->get_logger(), "Initial Pose: '%f, %f, %f'", this -> initial_pose_[0], this -> initial_pose_[1], this -> initial_pose_[2]); 
  }



