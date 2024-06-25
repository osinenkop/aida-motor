#include "MotorManager.hxx"

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <algorithm>
#include <regex>

std::string getNodeName(int argc, char* argv[]){
    const char* target_name{"LK_TECH"};
    char* entry{};
    std::for_each(argv + 1, argv + argc, [target_name, &entry](char* arg)
    {if(std::strstr(arg, target_name) != nullptr) entry = arg;});
    std::regex pattern(std::string(target_name) + "(.*)");
    std::cmatch match;
    return (std::regex_search(entry, match, pattern))?match[0].str():"";
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    std::string node_name(getNodeName(argc, argv));
    std::shared_ptr<MinimalMotor> md{std::make_shared<MinimalMotor>(node_name)};
  try {
    md->warmUp();
    std::clog << "spin" << std::endl;
    rclcpp::spin(md);
  } catch (std::exception &e) {
    std::cerr << "Caught Error " << e.what() << std::endl;
    rclcpp::shutdown();
  }
    rclcpp::shutdown();
  return 0;
}