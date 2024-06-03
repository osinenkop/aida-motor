#include "Base.hxx"

auto Base::initAccelerationCmd() -> void{
    this -> data_.read_acc_cmd_ = {0x3E, 0x33, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_acc_cmd_, 0, 3, this -> data_.read_acc_cmd_[4]);
    this -> data_.read_acc_response_.resize(10, 0x00);

    this -> data_.write_acc_cmd_ = {0x3E, 0x34, this->device_id_, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.write_acc_cmd_, 0, 3, this -> data_.write_acc_cmd_[4]);
}

auto Base::getAcceleration() -> void{
    /*The computer host sends this command to read the acceleration parameters of the current motor.*/

    this -> client_.send(this -> data_.read_acc_cmd_);
    this -> client_.receive(this -> data_.read_acc_response_);

    /*Acceleration parameters are included in the driver reply data. 
    The acceleration data is of int32_t type, with a unit of 1dps/s.*/
    this -> acceleration =  (
                            (this -> data_.read_acc_response_[5]      )   +       // *(uint8_t *)(&Accel)
                            (this -> data_.read_acc_response_[6] << 8 )   +       // *((uint8_t *)(&Accel)+1)
                            (this -> data_.read_acc_response_[7] << 16)   +       // *((uint8_t *)(&Accel)+2)
                            (this -> data_.read_acc_response_[8] << 24)           // *((uint8_t *)(&Accel)+3)
                            );
}

auto Base::setAcceleration(uint32_t& acceleration) -> void{
    /*The computer host sends the command to write acceleration parameters into RAM, 
    and the parameters will lose when power off. Acceleration data is int32_t type, unit 1dps/s.*/
    this -> data_.write_acc_cmd_[5] = ((acceleration      ) & 0xFF);      // *(uint8_t *)(&Accel)
    this -> data_.write_acc_cmd_[6] = ((acceleration >> 8 ) & 0xFF);      // *((uint8_t *)(&Accel)+1)
    this -> data_.write_acc_cmd_[7] = ((acceleration >> 16) & 0xFF);      // *((uint8_t *)(&Accel)+2)
    this -> data_.write_acc_cmd_[8] = ((acceleration >> 24) & 0xFF);      // *((uint8_t *)(&Accel)+3)

    this -> client_.calculateCheckSum(this -> data_.write_acc_cmd_, 5, 8, this -> data_.write_acc_cmd_[10]);
    
    this -> client_.send(this -> data_.write_acc_cmd_);
    usleep(1000000);
    this -> getAcceleration();
}

