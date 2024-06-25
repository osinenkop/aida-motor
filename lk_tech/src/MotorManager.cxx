#include "MotorManager.hxx"


MinimalMotor::MinimalMotor(std::string node_name): Node(node_name) 
  {
    // std::string node_name{"Hey there"};

    this -> declare_parameter("id", rclcpp::PARAMETER_STRING);
    std::string id = this->get_parameter("id").as_string();
    id.pop_back();

    this -> id_ = std::stoi(id);


    this -> declare_parameter("initial_pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this -> initial_pose_ = this->get_parameter("initial_pose").as_double_array();

    this -> declare_parameter("initial_orientation", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this -> initial_orientation_ = this->get_parameter("initial_orientation").as_double_array();

    this -> declare_parameter("port_list", rclcpp::PARAMETER_STRING_ARRAY);
    this -> port_list_ = this->get_parameter("port_list").as_string_array();


    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", node_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Port Addr: '%s'", this -> port_address_.c_str());
    RCLCPP_INFO(this->get_logger(), "Device ID: '%d'", this -> id_); 
    RCLCPP_INFO(this->get_logger(), "Initial Pose: '%f, %f, %f'", this -> initial_pose_[0], this -> initial_pose_[1], this -> initial_pose_[2]);
    RCLCPP_INFO(this->get_logger(), "Initial Orientation: '%f, %f, %f'", this -> initial_orientation_[0], this -> initial_orientation_[1], this -> initial_orientation_[2]); 
  }

MinimalMotor::~MinimalMotor(){
  this -> motor_ -> stop();
}

auto MinimalMotor::matchPort() -> void{
  for (const std::string& port: this -> port_list_){
    std::unique_ptr<Motor> m = std::make_unique<Motor>(port, this -> id_, true);
    if(m -> getProductInfo().size() > 0){
      this -> port_address_ = port;
      this -> motor_found_ = true;
      break;
    }
  }
}

auto MinimalMotor::warmUp() -> void{
  this -> matchPort();
  if (this -> motor_found_){
    this -> motor_ = std::make_unique<Motor>( this -> port_address_, this -> id_, false);
    this -> subscription_ = this->create_subscription<std_msgs::msg::Float32>("LK_TECH/motor_"+std::to_string(static_cast<unsigned>(this -> id_))+"/command", 1, std::bind(&MinimalMotor::callback, this, std::placeholders::_1));
    this -> publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("LK_TECH/motor_"+std::to_string(static_cast<unsigned>(this -> id_))+"/feedback", 1);

    this-> feedback_.data.resize(4);
    this -> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }
}


auto MinimalMotor::callback(std_msgs::msg::Float32::SharedPtr msg) -> void{
  this -> motor_ -> speedControl(msg -> data);
  
  this-> feedback_.data = { this -> motor_ -> temperature,
                            this -> motor_ -> torque,
                            this -> motor_ -> speed,
                            this -> motor_ -> position,
                          };
  
  if (not start_flag){
    start_value = this -> motor_ -> position;
    start_flag = true;
    start_time = std::chrono::steady_clock::now();
    max_val = this -> motor_ -> position;
  }


  this -> publisher_ -> publish(this-> feedback_);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "robot";
  t.child_frame_id = "motor_"+std::to_string(static_cast<unsigned>(this -> id_));

  t.transform.translation.x = this -> initial_pose_[0];
  t.transform.translation.y = this -> initial_pose_[1];
  t.transform.translation.z = this -> initial_pose_[2];

  tf2::Quaternion q;
  q.setRPY(this -> initial_orientation_[0], this -> initial_orientation_[1], 3.14 * this -> motor_ -> position / 180.0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  this -> tf_broadcaster_->sendTransform(t);
}



