#include "Base.hxx"

auto Base::initTorqueCmd() -> void{
    // this -> data_.read_temperature_and_voltage_cmd_ = {0x3E, 0x9A, this->device_id_, 0x00, 0x00};
    // this -> client_.calculateCheckSum(this -> data_.read_temperature_and_voltage_cmd_, 0, 3, this -> data_.read_temperature_and_voltage_cmd_[4]);
    // this -> data_.read_temperature_and_voltage_response_.resize(13, 0x00);

    // this -> data_.clear_error_cmd_ = {0x3E, 0x9B, this->device_id_, 0x00, 0x00};
    // this -> client_.calculateCheckSum(this -> data_.clear_error_cmd_, 0, 3, this -> data_.clear_error_cmd_[4]);
    // this -> data_.clear_error_response_.resize(13, 0x00);

    // this -> data_.read_temperature_and_torque_speed_pose_cmd_ = {0x3E, 0x9C, this->device_id_, 0x00, 0x00};
    // this -> client_.calculateCheckSum(this -> data_.read_temperature_and_torque_speed_pose_cmd_, 0, 3, this -> data_.read_temperature_and_torque_speed_pose_cmd_[4]);
    // this -> data_.read_temperature_and_torque_speed_pose_response_.resize(13, 0x00);

    // this -> data_.read_temperature_and_phase_current_cmd_ = {0x3E, 0x9D, this->device_id_, 0x00, 0x00};
    // this -> client_.calculateCheckSum(this -> data_.read_temperature_and_phase_current_cmd_, 0, 3, this -> data_.read_temperature_and_phase_current_cmd_[4]);
    // this -> data_.read_temperature_and_phase_current_response_.resize(13, 0x00);
}