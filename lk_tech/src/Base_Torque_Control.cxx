#include "Base.hxx"

auto Base::initTorqueControlCmd() -> void{
    this -> data_.open_loop_torque_cmd_ = {0x3E, 0xA0, this->device_id_, 0x02, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.open_loop_torque_cmd_, 0, 3, this -> data_.open_loop_torque_cmd_[4]);
    this -> data_.open_loop_torque_response_.resize(13, 0x00);

    this -> data_.closed_loop_torque_cmd_ = {0x3E, 0xA1, this->device_id_, 0x02, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.closed_loop_torque_cmd_, 0, 3, this -> data_.closed_loop_torque_cmd_[4]);
    this -> data_.closed_loop_torque_response_.resize(13, 0x00);
}


auto Base::openLoopTorqueControl(const std::int16_t& value) -> void{
    /*The computer host sends this command to control the output power of open loop, 
    and the control value is int16_t type, with the value range of -1000~ 1000, 
    (the bus current and the actual torque of the motor vary with different motors).*/

    this -> data_.open_loop_torque_cmd_[5] = ( value        & 0xFF);
    this -> data_.open_loop_torque_cmd_[6] = ((value >> 8)  & 0xFF);
    this -> client_.calculateCheckSum(this -> data_.open_loop_torque_cmd_, 5, 6, this -> data_.open_loop_torque_cmd_[7]);

    this -> client_.send(this -> data_.open_loop_torque_cmd_);
    this -> client_.receive(this -> data_.open_loop_torque_response_);

    /*
    The motor replies to the computer host after receiving the command, and the frame data contains the following data:
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Motor power output value (int16_t type, range -1000~1000)
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, eg:14bit encoder value range 0~16383).
    */

    this -> temperature = this -> data_.open_loop_torque_response_[5];

    this -> power     = (
                        (this -> data_.open_loop_torque_response_[6]            ) +
                        (this -> data_.open_loop_torque_response_[7] << 8       ) 
                        );


    this -> speed     = (
                        (this -> data_.open_loop_torque_response_[8]            ) +
                        (this -> data_.open_loop_torque_response_[9] << 8       ) 
                        );


    this -> position  = (
                        (this -> data_.open_loop_torque_response_[10]           ) +
                        (this -> data_.open_loop_torque_response_[11] << 8      ) 
                        );
}



auto Base::closedLoopTorqueControl(const std::int16_t& value) -> void{
    /*The computer host sends this command to control the torque current output of the motor, 
    and the control value is int16_t type, with the value range of -2000~ 2000, 
    corresponding to the actual torque current range of -32A ~32A 
    (the bus current and the actual torque of the motor vary with different motors).*/

    this -> data_.closed_loop_torque_cmd_[5] = ( value        & 0xFF);
    this -> data_.closed_loop_torque_cmd_[6] = ((value >> 8)  & 0xFF);
    this -> client_.calculateCheckSum(this -> data_.closed_loop_torque_cmd_, 5, 6, this -> data_.closed_loop_torque_cmd_[7]);
    
    this -> client_.send(this -> data_.closed_loop_torque_cmd_);
    this -> client_.receive(this -> data_.closed_loop_torque_response_);

    /*
    The motor replies to the computer host after receiving the command, and the frame data contains the following data:
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual torque current range -33A ~33A).
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, 14bit encoder value range 0~16383).
    */

    this -> temperature = this -> data_.open_loop_torque_response_[5];

    this -> torque    = (
                        (this -> data_.open_loop_torque_response_[6]            ) +
                        (this -> data_.open_loop_torque_response_[7] << 8       ) 
                        );


    this -> speed     = (
                        (this -> data_.open_loop_torque_response_[8]            ) +
                        (this -> data_.open_loop_torque_response_[9] << 8       ) 
                        );


    this -> position  = (
                        (this -> data_.open_loop_torque_response_[10]           ) +
                        (this -> data_.open_loop_torque_response_[11] << 8      ) 
                        );
}