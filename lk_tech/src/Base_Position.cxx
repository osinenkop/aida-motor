#include "Base.hxx"

auto Base::initPositionCmd() -> void{
    this -> data_.read_enc_cmd_ = {0x3E, 0x90, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_enc_cmd_, 0, 3, this -> data_.read_enc_cmd_[4]);
    this -> data_.read_enc_response_.resize(12, 0x00);

    this -> data_.write_temp_enc_offset_cmd_ = {0x3E, 0x91, this->device_id_, 0x02, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.write_temp_enc_offset_cmd_, 0, 3, this -> data_.write_temp_enc_offset_cmd_[4]);

    this -> data_.write_perm_enc_offset_cmd_ = {0x3E, 0x19, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.write_perm_enc_offset_cmd_, 0, 3, this -> data_.write_perm_enc_offset_cmd_[4]);

    this -> data_.read_multi_turn_cmd_ = {0x3E, 0x92, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_multi_turn_cmd_, 0, 3, this -> data_.read_multi_turn_cmd_[4]);
    this -> data_.read_multi_turn_response_.resize(14, 0x00);

    this -> data_.read_single_turn_cmd_ = {0x3E, 0x94, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_single_turn_cmd_, 0, 3, this -> data_.read_single_turn_cmd_[4]);
    this -> data_.read_single_turn_response_.resize(10, 0x00);
}


auto Base::getPosition() -> void{
    /*The computer host sends command to read the current position of the encoder.*/

    this -> client_.send(this -> data_.read_enc_cmd_);
    this -> client_.receive(this -> data_.read_enc_response_);

    /*The motor replies to the computer host after receiving the command, and the reply data contains the following parameters.
    1.Encoder position encoder (std::uint16_t type, eg:14bit encoder value range 0~16383), which is the original position of encoder minus encoder offset.
    2.Original position of encoder (std::uint16_t type, eg:14bit encoder value range 0~16383). 
    3.EncoderOffset (std::uint16_t type, eg:14bit encoder value range 0~16383), and this point is taken as the 0 point of the motor Angle.
    */


    this -> position        =   (
                                (static_cast<uint16_t>(this -> data_.read_enc_response_[5])       )     |
                                (static_cast<uint16_t>(this -> data_.read_enc_response_[6]) << 8  )
                                ) & this -> data_.encoder_mask_;

    this -> raw_position    =   (
                                (static_cast<uint16_t>(this -> data_.read_enc_response_[7])       )     |
                                (static_cast<uint16_t>(this -> data_.read_enc_response_[8]) << 8  )
                                ) & this -> data_.encoder_mask_;

    this -> position_offset =   (
                                (static_cast<uint16_t>(this -> data_.read_enc_response_[9] )      )     |
                                (static_cast<uint16_t>(this -> data_.read_enc_response_[10]) << 8 )
                                ) & this -> data_.encoder_mask_;
    
}


auto Base::setTemporaryPositionZero(std::uint16_t offset) -> void{
    /*The computer host sends the command to set the encoder Offset , 
    that the encoder Offset to be written is the type of std::uint16_t, and value range of the 14bit encoder is 0~16383.*/
    offset &= this -> data_.encoder_mask_;
    this -> data_.write_temp_enc_offset_cmd_[5] = ((offset          ) & 0xFF);     // *(std::uint8_t *)(&encoderOffset)
    this -> data_.write_temp_enc_offset_cmd_[6] = ((offset >> 8     ) & 0xFF);     // *((std::uint8_t *)(&encoderOffset)+1)
    
    this -> client_.calculateCheckSum(this -> data_.write_temp_enc_offset_cmd_, 5, 6, this -> data_.write_temp_enc_offset_cmd_[7]);

    this -> client_.send(this -> data_.write_temp_enc_offset_cmd_);
    usleep(1000000);
    this -> getPosition();
}


auto Base::setPermanentPositionZero() -> void{
    /*Writes the current encoder position of the motor into ROM as the initial position.
    Attention:
    1. This command needs to restart to take effect.
    2. This command will write zero point into ROM of the driver, multiple writing will affect the chip life, which is not recommended for frequent use
    */

    this -> client_.send(this -> data_.write_perm_enc_offset_cmd_);
    usleep(1000000);
    this -> getPosition();
}


auto Base::getMultiTurnAngle() -> void{
    /*The computer host sends command to read the absolute multi-turn Angle of the current motor.*/

    this -> client_.send(this -> data_.read_multi_turn_cmd_);
    this -> client_.receive(this -> data_.read_multi_turn_response_);

    /*The motor replies to the computer host after receiving the command, 
    and the frame data contains the following parameters:
    1.Motor-angle, int64_t type data, positive value represents clockwise cumulative Angle, 
    negative value represents counter clockwise cumulative Angle, unit 0.01°/LSB.*/

    this -> multi_turn_angle  =     (
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[5])           ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[6])   << 8    ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[7])   << 16   ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[8])   << 24   ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[9])   << 32   ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[10])  << 40   ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[11])  << 48   ) |
                                    (static_cast<int64_t>(this -> data_.read_multi_turn_response_[12])  << 56   ) 
                                    );
}

auto Base::getSingleTurnAngle() -> void{
    /*The computer host sends command to read the absolute single-turn Angle of the current motor.*/

    this -> client_.send(this -> data_.read_single_turn_cmd_);
    this -> client_.receive(this -> data_.read_single_turn_response_);

    /*The motor replies to the computer host after receiving the command, the frame data contains the following parameters:
    1.The single-loop angle of the motor,uint32_t type data, 
    which takes encoder zero point as the starting point, increases clockwise, 
    and when it reaches zero again, the value returns to 0, unit 0.01°/LSB, 
    and the value range is 0~36000*i-1(i:Reduction ratio).*/

    this -> single_turn_angle  =    (
                                    (static_cast<uint32_t>(this -> data_.read_single_turn_response_[5])           ) |
                                    (static_cast<uint32_t>(this -> data_.read_single_turn_response_[6])   << 8    ) |
                                    (static_cast<uint32_t>(this -> data_.read_single_turn_response_[7])   << 16   ) |
                                    (static_cast<uint32_t>(this -> data_.read_single_turn_response_[8])   << 24   ) 
                                    );
}




