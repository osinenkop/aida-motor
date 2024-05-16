#include "Motor.hxx"

auto Motor::getPID() -> void{
    this -> client_.send(this -> read_pid_cmd_);
    this -> client_.receive(this -> read_pid_response_);
    this -> removeSpace(this -> read_pid_response_);

    // Data Format
    // Position_Kp, Position_Ki, Speed_Kp, Speed_Ki, Torque_Kp, Torque_Ki
    std::copy(this -> response_str.begin()+5, this -> response_str.begin()+11, this -> pid_value.begin());
}

auto Motor::setTemporaryPID(const std::array<uint8_t, 6>& pid) -> void{
    std::copy(pid.begin(), pid.end(), this -> write_pid_ram_cmd_.begin()+5);
    this -> client_.calculateCheckSum(this -> write_pid_ram_cmd_, 5, 10, this -> write_pid_ram_cmd_[11]);
    
    this -> client_.send(this -> write_pid_ram_cmd_);
    this -> getPID();
}

auto Motor::setTemporaryPID(const std::array<uint8_t, 6>&& pid) -> void{
    std::copy(pid.begin(), pid.end(), this -> write_pid_ram_cmd_.begin()+5);
    this -> client_.calculateCheckSum(this -> write_pid_ram_cmd_, 5, 10, this -> write_pid_ram_cmd_[11]);
    
    this -> client_.send(this -> write_pid_ram_cmd_);
    this -> getPID();
}

auto Motor::setPermanentPID(const std::array<uint8_t, 6>& pid) -> void{
    std::copy(pid.begin(), pid.end(), this -> write_pid_rom_cmd_.begin()+5);
    this -> client_.calculateCheckSum(this -> write_pid_rom_cmd_, 5, 10, this -> write_pid_rom_cmd_[11]);

    this -> client_.send(this -> write_pid_rom_cmd_);
    usleep(1000000);
    this -> getPID();
}

auto Motor::setPermanentPID(const std::array<uint8_t, 6>&& pid) -> void{
    std::copy(pid.begin(), pid.end(), this -> write_pid_rom_cmd_.begin()+5);
    this -> client_.calculateCheckSum(this -> write_pid_rom_cmd_, 5, 10, this -> write_pid_rom_cmd_[11]);
    
    this -> client_.send(this -> write_pid_rom_cmd_);
    usleep(1000000);
    this -> getPID();
}