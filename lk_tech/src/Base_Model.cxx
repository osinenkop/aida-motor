#include "Base.hxx"

auto Base::initModelCmd() -> void{
    this -> data_.model_cmd_ = {0x3E, 0x12, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.model_cmd_, 0, 3, this -> data_.model_cmd_[4]);
    // this -> model_response_ = std::vector<uint8_t>(48);
    this -> data_.model_response_.resize(255, 0x00);
}

auto Base::getModel() -> void{
    this -> client_.send(this -> data_.model_cmd_);
    this -> client_.receive(this -> data_.model_response_);
    this -> removeSpace(this -> data_.model_response_);

    this -> model = this -> response_str.substr(5, 19);
}