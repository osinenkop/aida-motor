#include "Base.hxx"

auto Base::initSpeedControlCmd() -> void{
    this -> data_.closed_loop_speed_cmd_ = {0x3E, 0xA2, this->device_id_, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.closed_loop_speed_cmd_, 0, 3, this -> data_.closed_loop_speed_cmd_[4]);
    this -> data_.closed_loop_speed_response_.resize(13, 0x00);
}

auto Base::closedLoopSpeedControl(const std::int32_t& value) -> void{
    /*The computer host sends this command to control the speed of the motor with a speedControl 
    of type int32_t corresponding to the actual speed of 0.01 DPS /LSB.*/

    this -> data_.closed_loop_speed_cmd_[5] = ((value           ) & 0xFF);
    this -> data_.closed_loop_speed_cmd_[6] = ((value >>  8     ) & 0xFF);
    this -> data_.closed_loop_speed_cmd_[7] = ((value >> 16     ) & 0xFF);
    this -> data_.closed_loop_speed_cmd_[8] = ((value >> 24     ) & 0xFF);

    this -> client_.calculateCheckSum(this -> data_.closed_loop_speed_cmd_, 5, 8, this -> data_.closed_loop_speed_cmd_[9]);

    this -> client_.send(this -> data_.closed_loop_speed_cmd_);
    this -> client_.receive(this -> data_.closed_loop_speed_response_);

    /*The motor replies to the computer host after receiving the command, and the frame data contains the following data.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual torque current range -33A ~33A). 
    3. Motor power output value (int16_t type, range -1000~1000) 3.Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, 14bit encoder value range 0~16383).*/

    this -> temperature = this -> data_.closed_loop_speed_response_[5];

    this -> torque    = (
                        (static_cast<int16_t>(this -> data_.closed_loop_speed_response_[6])            ) |
                        (static_cast<int16_t>(this -> data_.closed_loop_speed_response_[7]) << 8       ) 
                        );


    this -> speed     = (
                        (static_cast<int16_t>(this -> data_.closed_loop_speed_response_[8])            ) |
                        (static_cast<int16_t>(this -> data_.closed_loop_speed_response_[9]) << 8       ) 
                        );


    this -> position  = (
                        (static_cast<uint16_t>(this -> data_.closed_loop_speed_response_[10])           ) |
                        (static_cast<uint16_t>(this -> data_.closed_loop_speed_response_[11]) << 8      ) 
                        ) & this -> data_.encoder_mask_;
}

