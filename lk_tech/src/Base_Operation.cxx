#include "Base.hxx"

auto Base::initOperationCmd() -> void{
    this -> data_.shutdown_cmd_ = {0x3E, 0x80, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.shutdown_cmd_, 0, 3, this -> data_.shutdown_cmd_[4]);
    this -> data_.shutdown_response_.resize(5, 0x00);

    this -> data_.stop_cmd_ = {0x3E, 0x81, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.stop_cmd_, 0, 3, this -> data_.stop_cmd_[4]);
    this -> data_.stop_response_.resize(5, 0x00);

    this -> data_.operate_cmd_ = {0x3E, 0x88, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.operate_cmd_, 0, 3, this -> data_.operate_cmd_[4]);
    this -> data_.operate_response_.resize(5, 0x00);
}


auto Base::shutdown() -> void{
    /*Turn off the motor and clear the motor running state and the control instruction received before.*/

    this -> client_.send(this -> data_.shutdown_cmd_);
    this -> client_.receive(this -> data_.shutdown_response_);

    /*On successful attempt, the response should be the same as the command*/
    this -> shutdown_success = (this -> data_.shutdown_cmd_ == this -> data_.shutdown_response_);
}

auto Base::stop() -> void{
    /*Stop the motor, but do not clear the motor running state and previous received control instructions.*/

    this -> client_.send(this -> data_.stop_cmd_);
    this -> client_.receive(this -> data_.stop_response_);

    /*On successful attempt, the response should be the same as the command*/
    this -> stop_success = (this -> data_.stop_cmd_ == this -> data_.stop_response_);
}

auto Base::operate() -> void{
    /*Restore motor operation from motor stop command (control mode before restoration stop).*/

    this -> client_.send(this -> data_.operate_cmd_);
    this -> client_.receive(this -> data_.operate_response_);

    /*On successful attempt, the response should be the same as the command*/
    this -> operate_success = (this -> data_.operate_cmd_ == this -> data_.operate_response_);
}