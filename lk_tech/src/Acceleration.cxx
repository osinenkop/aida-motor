#include "Motor.hxx"

auto Motor::getAcceleration() -> void{
    this -> client_.send(this -> read_acc_cmd_);
    this -> client_.receive(this -> read_acc_response_);
    this -> removeSpace(this -> read_acc_response_);

    // for(int i: this -> read_acc_response_) std::cout << std::hex << i << " ";
    // std::cout << "\n";

    this -> acceleration =  (
                            (read_acc_response_[5]      )   +
                            (read_acc_response_[6] << 8 )   + 
                            (read_acc_response_[7] << 16)   + 
                            (read_acc_response_[8] << 24)
                            );
}

auto Motor::setAcceleration(uint32_t& acceleration) -> void{
    
    this -> write_acc_cmd_[5] = (acceleration         & 0xFF);
    this -> write_acc_cmd_[6] = ((acceleration >> 8)  & 0xFF);
    this -> write_acc_cmd_[7] = ((acceleration >> 16) & 0xFF);
    this -> write_acc_cmd_[8] = ((acceleration >> 24) & 0xFF);
    this -> client_.calculateCheckSum(this -> write_acc_cmd_, 5, 8, this -> write_acc_cmd_[10]);
    
    this -> client_.send(this -> write_acc_cmd_);
    usleep(1000000);
    this -> getAcceleration();
}

auto Motor::setAcceleration(uint32_t&& acceleration) -> void{
    
    this -> write_acc_cmd_[5] = (acceleration         & 0xFF);
    this -> write_acc_cmd_[6] = ((acceleration >> 8)  & 0xFF);
    this -> write_acc_cmd_[7] = ((acceleration >> 16) & 0xFF);
    this -> write_acc_cmd_[8] = ((acceleration >> 24) & 0xFF);
    this -> client_.calculateCheckSum(this -> write_acc_cmd_, 5, 8, this -> write_acc_cmd_[10]);
    
    this -> client_.send(this -> write_acc_cmd_);
    usleep(1000000);
    this -> getAcceleration();
}