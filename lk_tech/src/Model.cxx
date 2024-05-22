#include "Motor.hxx"

auto Motor::initModelCmd() -> void{
    this -> model_cmd_ = {0x3E, 0x12, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> model_cmd_, 0, 3, this -> model_cmd_[4]);
    // this -> model_response_ = std::vector<uint8_t>(48);
    this -> model_response_.resize(255, 0x00);
}

auto Motor::getModel() -> void{
    this -> client_.send(this -> model_cmd_);
    this -> client_.receive(this -> model_response_);
    this -> removeSpace(this -> model_response_);

    this -> model = this -> response_str.substr(5, 19);
}