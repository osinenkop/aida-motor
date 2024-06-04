#include "Base.hxx"
#include <cstring>

auto Base::initModelCmd() -> void{
    this -> data_.model_cmd_ = {0x3E, 0x12, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.model_cmd_, 0, 3, this -> data_.model_cmd_[4]);
    this -> data_.model_response_.resize(64, 0x00);
}

auto Base::getModel() -> void{
    this -> client_.send(this -> data_.model_cmd_);
    this -> client_.receive(this -> data_.model_response_);

    std::memcpy(this -> product_info.driver_name, &this -> data_.model_response_[5],      20);
    std::memcpy(this -> product_info.motor_name,  &this -> data_.model_response_[5 + 20], 20);
    this -> product_info.hardware_version = this -> data_.model_response_[45];
    this -> product_info.firmware_version = this -> data_.model_response_[46];

    // printf("driver_name: %s\n",         this -> product_info.driver_name);
    // printf("motor_name: %s\n",          this -> product_info.motor_name);
    // printf("hardware_version: %c\n",    this -> product_info.hardware_version);
    // printf("firmware_version: %c\n\n",  this -> product_info.firmware_version);
}

