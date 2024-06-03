#include "Base.hxx"

auto Base::initPositionControlCmd() -> void{
    this -> data_.closed_loop_multi_turn_position_1_cmd_ = {0x3E, 0xA3, this->device_id_, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.closed_loop_multi_turn_position_1_cmd_, 0, 3, this -> data_.closed_loop_multi_turn_position_1_cmd_[4]);
    this -> data_.closed_loop_multi_turn_position_1_response_.resize(13, 0x00);

    this -> data_.closed_loop_multi_turn_position_2_cmd_ = {0x3E, 0xA4, this->device_id_, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.closed_loop_multi_turn_position_2_cmd_, 0, 3, this -> data_.closed_loop_multi_turn_position_2_cmd_[4]);
    this -> data_.closed_loop_multi_turn_position_2_response_.resize(13, 0x00);

    this -> data_.closed_loop_single_turn_position_1_cmd_ = {0x3E, 0xA5, this->device_id_, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.closed_loop_single_turn_position_1_cmd_, 0, 3, this -> data_.closed_loop_single_turn_position_1_cmd_[4]);
    this -> data_.closed_loop_single_turn_position_1_response_.resize(13, 0x00);

    this -> data_.closed_loop_single_turn_position_2_cmd_ = {0x3E, 0xA6, this->device_id_, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.closed_loop_single_turn_position_2_cmd_, 0, 3, this -> data_.closed_loop_single_turn_position_2_cmd_[4]);
    this -> data_.closed_loop_single_turn_position_2_response_.resize(13, 0x00);
}

auto Base::closedLoopMultiPositionControl(const std::int64_t& value) -> void{
    /*The host computer sends the command to control the position of the motor (multi-turn Angle), 
    the control value angleControl is int64_t, corresponding to the actual position is 0.01degree/LSB, 
    that is 36000 represents 360°, and the motor rotation direction is determined 
    by the difference between the target position and the current position.*/

    this -> data_.closed_loop_multi_turn_position_1_cmd_[5]  =  ((value           ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[6]  =  ((value   >> 8    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[7]  =  ((value   >> 16   ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[8]  =  ((value   >> 24   ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[9]  =  ((value   >> 32   ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[10] =  ((value   >> 40   ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[11] =  ((value   >> 48   ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_1_cmd_[12] =  ((value   >> 56   ) & 0xFF);

    this -> client_.calculateCheckSum(this -> data_.closed_loop_multi_turn_position_1_cmd_, 5, 12, this -> data_.closed_loop_multi_turn_position_1_cmd_[13]);

    this -> client_.send(this -> data_.closed_loop_multi_turn_position_1_cmd_);
    this -> client_.receive(this -> data_.closed_loop_multi_turn_position_1_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following data.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual torque current range -33A ~33A); Motor power output value (int16_t type, range -1000~1000) 
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, 14bit encoder value range 0~16383).*/

    this -> temperature = this -> data_.closed_loop_multi_turn_position_1_response_[5];

    this -> torque    = (
                        (this -> data_.closed_loop_multi_turn_position_1_response_[6]            ) +
                        (this -> data_.closed_loop_multi_turn_position_1_response_[7] << 8       ) 
                        );


    this -> speed     = (
                        (this -> data_.closed_loop_multi_turn_position_1_response_[8]            ) +
                        (this -> data_.closed_loop_multi_turn_position_1_response_[9] << 8       ) 
                        );


    this -> position  = (
                        (this -> data_.closed_loop_multi_turn_position_1_response_[10]           ) +
                        (this -> data_.closed_loop_multi_turn_position_1_response_[11] << 8      ) 
                        );
}

auto Base::closedLoopMultiPositionControl(const std::int64_t& value, const std::uint32_t& max_speed) -> void{
    /*The host computer sends the command to control the position of the motor (multi-turn Angle), 
    the control value angleControl is int64_t, corresponding to the actual position is 0.01degree/LSB, 
    that is 36000 represents 360°, and the motor rotation direction is determined by the difference 
    between the target position and the current position. The control value maxSpeed limits the maximum
    speed of motor rotation, which is uint32_t type, corresponding to the actual speed of 0.01dps/LSB, 
    namely 36000 represents 360dps.*/

    this -> data_.closed_loop_multi_turn_position_2_cmd_[5]  =  ((value                 ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[6]  =  ((value        >> 8     ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[7]  =  ((value        >> 16    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[8]  =  ((value        >> 24    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[9]  =  ((value        >> 32    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[10] =  ((value        >> 40    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[11] =  ((value        >> 48    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[12] =  ((value        >> 56    ) & 0xFF);

    this -> data_.closed_loop_multi_turn_position_2_cmd_[13] =  ((max_speed             ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[14] =  ((max_speed    >> 8     ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[15] =  ((max_speed    >> 16    ) & 0xFF);
    this -> data_.closed_loop_multi_turn_position_2_cmd_[16] =  ((max_speed    >> 24    ) & 0xFF);

    this -> client_.calculateCheckSum(this -> data_.closed_loop_multi_turn_position_2_cmd_, 5, 16, this -> data_.closed_loop_multi_turn_position_2_cmd_[17]);

    this -> client_.send(this -> data_.closed_loop_multi_turn_position_2_cmd_);
    this -> client_.receive(this -> data_.closed_loop_multi_turn_position_2_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following data.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual torque current range -33A ~33A). Motor power output value (int16_t type, range -1000~1000) 
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, 14bit encoder value range 0~16383).*/

    this -> temperature = this -> data_.closed_loop_multi_turn_position_2_response_[5];

    this -> torque    = (
                        (this -> data_.closed_loop_multi_turn_position_2_response_[6]            ) +
                        (this -> data_.closed_loop_multi_turn_position_2_response_[7] << 8       ) 
                        );


    this -> speed     = (
                        (this -> data_.closed_loop_multi_turn_position_2_response_[8]            ) +
                        (this -> data_.closed_loop_multi_turn_position_2_response_[9] << 8       ) 
                        );


    this -> position  = (
                        (this -> data_.closed_loop_multi_turn_position_2_response_[10]           ) +
                        (this -> data_.closed_loop_multi_turn_position_2_response_[11] << 8      ) 
                        );
}


auto Base::closedLoopSinglePositionControl(const std::uint16_t& value, const std::int8_t& spin_direction) -> void{
    /*1. The host sends this command to control the position of the motor (single turn Angle).
    2. The control value angleControl is uint16_t, with a range of 0~35999 and a corresponding actual 
    position of 0.01degree/LSB, namely the actual Angle range of 0°~359.99°.*/

    this -> data_.closed_loop_single_turn_position_1_cmd_[5]  =  spin_direction;
    this -> data_.closed_loop_single_turn_position_1_cmd_[6]  =  ((value           ) & 0xFF);
    this -> data_.closed_loop_single_turn_position_1_cmd_[7]  =  ((value   >> 8    ) & 0xFF);

    this -> client_.calculateCheckSum(this -> data_.closed_loop_single_turn_position_1_cmd_, 5, 8, this -> data_.closed_loop_single_turn_position_1_cmd_[9]);

    this -> client_.send(this -> data_.closed_loop_single_turn_position_1_cmd_);
    this -> client_.receive(this -> data_.closed_loop_single_turn_position_1_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following data.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual torque current range -33A ~33A). Motor power output value (int16_t type, range -1000~1000) 
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, 14bit encoder value range 0~16383).*/


    this -> temperature = this -> data_.closed_loop_single_turn_position_1_response_[5];

    this -> torque    = (
                        (this -> data_.closed_loop_single_turn_position_1_response_[6]            ) +
                        (this -> data_.closed_loop_single_turn_position_1_response_[7] << 8       ) 
                        );


    this -> speed     = (
                        (this -> data_.closed_loop_single_turn_position_1_response_[8]            ) +
                        (this -> data_.closed_loop_single_turn_position_1_response_[9] << 8       ) 
                        );


    this -> position  = (
                        (this -> data_.closed_loop_single_turn_position_1_response_[10]           ) +
                        (this -> data_.closed_loop_single_turn_position_1_response_[11] << 8      ) 
                        );
}


auto Base::closedLoopSinglePositionControl(const std::uint16_t& value, const std::int8_t& spin_direction, const std::uint32_t& max_speed) -> void{
    /*The computer host sends this command to control the position of the motor (single turn Angle). 
    1. Control value spinDirection sets the direction of motor rotation as uint8_t type, 0x00 represents clockwise, 0x01 represents counterclockwise.
    2. The control value angleControl is uint16_t, with a range of 0~35999 and a corresponding actual position of 0.01degree/LSB, namely the actual Angle range of 0°~359.99°.
    3. The control value maxSpeed limits the maximum speed of motor rotation, which is uint32_t type, corresponding to the actual speed of 0.01dps/LSB, i.e. 36000 represents 360dps.*/

    this -> data_.closed_loop_single_turn_position_2_cmd_[5]   =  spin_direction;
    this -> data_.closed_loop_single_turn_position_2_cmd_[6]   =  ((value             ) & 0xFF);
    this -> data_.closed_loop_single_turn_position_2_cmd_[7]   =  ((value   >> 8      ) & 0xFF);

    this -> data_.closed_loop_single_turn_position_2_cmd_[9]   =  ((value            ) & 0xFF);
    this -> data_.closed_loop_single_turn_position_2_cmd_[10]  =  ((value   >> 8     ) & 0xFF);
    this -> data_.closed_loop_single_turn_position_2_cmd_[11]  =  ((value   >> 16    ) & 0xFF);
    this -> data_.closed_loop_single_turn_position_2_cmd_[12]  =  ((value   >> 24    ) & 0xFF);
    this -> client_.calculateCheckSum(this -> data_.closed_loop_single_turn_position_2_cmd_, 5, 12, this -> data_.closed_loop_single_turn_position_2_cmd_[13]);

    this -> client_.send(this -> data_.closed_loop_single_turn_position_2_cmd_);
    this -> client_.receive(this -> data_.closed_loop_single_turn_position_2_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following data.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual torque current range -33A ~33A). Motor power output value (int16_t type, range -1000~1000) 
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, 14bit encoder value range 0~16383).*/


    this -> temperature = this -> data_.closed_loop_single_turn_position_2_response_[5];

    this -> torque    = (
                        (this -> data_.closed_loop_single_turn_position_2_response_[6]            ) +
                        (this -> data_.closed_loop_single_turn_position_2_response_[7] << 8       ) 
                        );


    this -> speed     = (
                        (this -> data_.closed_loop_single_turn_position_2_response_[8]            ) +
                        (this -> data_.closed_loop_single_turn_position_2_response_[9] << 8       ) 
                        );


    this -> position  = (
                        (this -> data_.closed_loop_single_turn_position_2_response_[10]           ) +
                        (this -> data_.closed_loop_single_turn_position_2_response_[11] << 8      ) 
                        );
}