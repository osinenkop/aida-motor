#include "Motor.hxx"


Motor::~Motor(){}

Motor::Motor(std::string port_address, uint8_t device_id): device_id_{device_id}, client_{PortHandler(port_address)}{

    this -> response_str.resize(255);

    this -> model_cmd_ = {0x3E, 0x12, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> model_cmd_, 0, 3, this -> model_cmd_[4]);
    // this -> model_response_ = std::vector<uint8_t>(48);
    this -> model_response_.resize(255, 0x00);


    this -> read_pid_cmd_ = {0x3E, 0x30, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> read_pid_cmd_, 0, 3, this -> read_pid_cmd_[4]);
    // this -> read_pid_response_ = std::vector<uint8_t>(12);
    this -> read_pid_response_.resize(255, 0x00);

    this -> write_pid_ram_cmd_ = {0x3E, 0x31, this->device_id_, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> write_pid_ram_cmd_, 0, 3, this -> write_pid_ram_cmd_[4]);

    this -> write_pid_rom_cmd_ = {0x3E, 0x32, this->device_id_, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> write_pid_rom_cmd_, 0, 3, this -> write_pid_rom_cmd_[4]);

    this -> read_acc_cmd_ = {0x3E, 0x33, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> read_acc_cmd_, 0, 3, this -> read_acc_cmd_[4]);
    // this -> read_acc_response_ = std::vector<uint8_t>(10);
    this -> read_acc_response_.resize(255, 0x00);


    this -> write_acc_cmd_ = {0x3E, 0x34, this->device_id_, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> write_acc_cmd_, 0, 3, this -> write_acc_cmd_[4]);

}


auto Motor::getModel() -> void{
    this -> client_.send(this -> model_cmd_);
    this -> client_.receive(this -> model_response_);
    this -> removeSpace(this -> model_response_);

    this -> model = this -> response_str.substr(5, 19);
}



auto Motor::removeSpace(std::vector<uint8_t>& arr) -> void{
    // this -> response_str.clear();
    std::copy_if(arr.begin(), arr.end(),
                    // std::back_inserter(this -> response_str),
                    this -> response_str.begin(),
                    [](uint8_t x) { return '\0' != x; });
}






