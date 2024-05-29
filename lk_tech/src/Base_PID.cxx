#include "Base.hxx"


auto Base::initPIDCmd() -> void{
    this -> data_.read_pid_cmd_ = {0x3E, 0x30, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_pid_cmd_, 0, 3, this -> data_.read_pid_cmd_[4]);
    this -> data_.read_pid_response_.resize(12, 0x00);

    this -> data_.write_pid_ram_cmd_ = {0x3E, 0x31, this->device_id_, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.write_pid_ram_cmd_, 0, 3, this -> data_.write_pid_ram_cmd_[4]);

    this -> data_.write_pid_rom_cmd_ = {0x3E, 0x32, this->device_id_, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.write_pid_rom_cmd_, 0, 3, this -> data_.write_pid_rom_cmd_[4]);
}

auto Base::getPID() -> void{
    /*The computer host sends command to read the PID parameter of the current motor. */

    this -> client_.send(this -> data_.read_pid_cmd_);
    this -> client_.receive(this -> data_.read_pid_response_);

    // // Position_Kp, Position_Ki, Speed_Kp, Speed_Ki, Torque_Kp, Torque_Ki
    // std::copy(this -> response_str.begin()+5, this -> response_str.begin()+11, this -> pid_value.begin());
    
    this -> pid_value[0] = this -> data_.read_pid_response_[5];     // Position_Kp
    this -> pid_value[1] = this -> data_.read_pid_response_[6];     // Position_Ki
    this -> pid_value[2] = this -> data_.read_pid_response_[7];     // Speed_Kp
    this -> pid_value[3] = this -> data_.read_pid_response_[8];     // Speed_Ki
    this -> pid_value[4] = this -> data_.read_pid_response_[9];     // Torque_Kp
    this -> pid_value[5] = this -> data_.read_pid_response_[10];    // Torque_Ki
}


auto Base::setTemporaryPID(const std::array<uint8_t, 6>& pid) -> void{
    /*The computer host sends command to write PID parameters into RAM, and the parameters become invalid when power off.*/

    // std::copy(pid.begin(), pid.end(), this -> write_pid_ram_cmd_.begin()+5);
    this -> data_.write_pid_ram_cmd_[5]   = pid[0];       // Position_Kp
    this -> data_.write_pid_ram_cmd_[6]   = pid[1];       // Position_Ki
    this -> data_.write_pid_ram_cmd_[7]   = pid[2];       // Speed_Kp
    this -> data_.write_pid_ram_cmd_[8]   = pid[3];       // Speed_Ki
    this -> data_.write_pid_ram_cmd_[9]   = pid[4];       // Torque_Kp
    this -> data_.write_pid_ram_cmd_[10]  = pid[5];       // Torque_Ki

    this -> client_.calculateCheckSum(this -> data_.write_pid_ram_cmd_, 5, 10, this -> data_.write_pid_ram_cmd_[11]);
    
    this -> client_.send(this -> data_.write_pid_ram_cmd_);
    this -> getPID();
}

auto Base::setTemporaryPID(const std::array<uint8_t, 6>&& pid) -> void{
    /*The computer host sends command to write PID parameters into RAM, and the parameters become invalid when power off.*/

    // std::copy(pid.begin(), pid.end(), this -> write_pid_ram_cmd_.begin()+5);
    this -> data_.write_pid_ram_cmd_[5]   = pid[0];       // Position_Kp
    this -> data_.write_pid_ram_cmd_[6]   = pid[1];       // Position_Ki
    this -> data_.write_pid_ram_cmd_[7]   = pid[2];       // Speed_Kp
    this -> data_.write_pid_ram_cmd_[8]   = pid[3];       // Speed_Ki
    this -> data_.write_pid_ram_cmd_[9]   = pid[4];       // Torque_Kp
    this -> data_.write_pid_ram_cmd_[10]  = pid[5];       // Torque_Ki

    this -> client_.calculateCheckSum(this -> data_.write_pid_ram_cmd_, 5, 10, this -> data_.write_pid_ram_cmd_[11]);
    
    this -> client_.send(this -> data_.write_pid_ram_cmd_);
    this -> getPID();
}

auto Base::setPermanentPID(const std::array<uint8_t, 6>& pid) -> void{
    /*The computer host sends the command to write the PID parameter to RAM. It is still valid when power off.*/

    // std::copy(pid.begin(), pid.end(), this -> write_pid_rom_cmd_.begin()+5);
    this -> data_.write_pid_rom_cmd_[5]   = pid[0];       // Position_Kp
    this -> data_.write_pid_rom_cmd_[6]   = pid[1];       // Position_Ki
    this -> data_.write_pid_rom_cmd_[7]   = pid[2];       // Speed_Kp
    this -> data_.write_pid_rom_cmd_[8]   = pid[3];       // Speed_Ki
    this -> data_.write_pid_rom_cmd_[9]   = pid[4];       // Torque_Kp
    this -> data_.write_pid_rom_cmd_[10]  = pid[5];       // Torque_Ki

    this -> client_.calculateCheckSum(this -> data_.write_pid_rom_cmd_, 5, 10, this -> data_.write_pid_rom_cmd_[11]);

    this -> client_.send(this -> data_.write_pid_rom_cmd_);
    usleep(1000000);
    this -> getPID();
}

auto Base::setPermanentPID(const std::array<uint8_t, 6>&& pid) -> void{
    /*The computer host sends the command to write the PID parameter to RAM. It is still valid when power off.*/

    // std::copy(pid.begin(), pid.end(), this -> write_pid_rom_cmd_.begin()+5);
    this -> data_.write_pid_rom_cmd_[5]   = pid[0];       // Position_Kp
    this -> data_.write_pid_rom_cmd_[6]   = pid[1];       // Position_Ki
    this -> data_.write_pid_rom_cmd_[7]   = pid[2];       // Speed_Kp
    this -> data_.write_pid_rom_cmd_[8]   = pid[3];       // Speed_Ki
    this -> data_.write_pid_rom_cmd_[9]   = pid[4];       // Torque_Kp
    this -> data_.write_pid_rom_cmd_[10]  = pid[5];       // Torque_Ki

    this -> client_.calculateCheckSum(this -> data_.write_pid_rom_cmd_, 5, 10, this -> data_.write_pid_rom_cmd_[11]);
    
    this -> client_.send(this -> data_.write_pid_rom_cmd_);
    usleep(1000000);
    this -> getPID();
}