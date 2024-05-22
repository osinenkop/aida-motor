#include "Motor.hxx"

auto Motor::initPositionCmd() -> void{
    this -> read_enc_cmd_ = {0x3E, 0x90, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> read_enc_cmd_, 0, 3, this -> read_enc_cmd_[4]);
    this -> read_enc_response_.resize(255, 0x00);

    this -> write_temp_enc_offset_cmd_ = {0x3E, 0x91, this->device_id_, 0x02, 0x00, 0x00, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> write_temp_enc_offset_cmd_, 0, 3, this -> write_temp_enc_offset_cmd_[4]);


    this -> write_perm_enc_offset_cmd_ = {0x3E, 0x19, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> write_perm_enc_offset_cmd_, 0, 3, this -> write_perm_enc_offset_cmd_[4]);
}


auto Motor::getPosition() -> void{
    /*The computer host sends command to read the current position of the encoder.*/

    this -> client_.send(this -> read_enc_cmd_);
    this -> client_.receive(this -> read_enc_response_);
    this -> removeSpace(this -> read_enc_response_);

    /*The motor replies to the computer host after receiving the command, and the reply data contains the following parameters.
    1.Encoder position encoder (uint16_t type, eg:14bit encoder value range 0~16383), which is the original position of encoder minus encoder offset.
    2.Original position of encoder (uint16_t type, eg:14bit encoder value range 0~16383). 
    3.EncoderOffset (uint16_t type, eg:14bit encoder value range 0~16383), and this point is taken as the 0 point of the motor Angle.
    */

    /* To get to understandable values,
    1- multiply by 360
    2- devide by 2^14
    -> The data will be expressed as degrees from 0-360
    -> apply the steps above on the data you receive from this function.
    */

    this -> position        =   (
                                (this -> read_enc_response_[5]       )     +
                                (this -> read_enc_response_[6] << 8  )
                                ) & encoder_mask_;

    this -> raw_position    =   (
                                (this -> read_enc_response_[7]       )     +
                                (this -> read_enc_response_[8] << 8  )
                                ) & encoder_mask_;

    this -> position_offset =   (
                                (this -> read_enc_response_[9]       )     +
                                (this -> read_enc_response_[10] << 8 )
                                ) & encoder_mask_;
    
}


auto Motor::setTemporaryPositionZero(uint16_t& offset) -> void{
    /*The computer host sends the command to set the encoder Offset , 
    that the encoder Offset to be written is the type of uint16_t, and value range of the 14bit encoder is 0~16383.*/
    offset &= encoder_mask_;
    this -> write_temp_enc_offset_cmd_[5] = ((offset          ) & 0xFF);     // *(uint8_t *)(&encoderOffset)
    this -> write_temp_enc_offset_cmd_[6] = ((offset >> 8     ) & 0xFF);     // *((uint8_t *)(&encoderOffset)+1)
    
    this -> client_.calculateCheckSum(this -> write_temp_enc_offset_cmd_, 5, 6, this -> write_temp_enc_offset_cmd_[7]);

    this -> client_.send(this -> write_temp_enc_offset_cmd_);
    usleep(1000000);
    this -> getPosition();
}


auto Motor::setTemporaryPositionZero(uint16_t&& offset) -> void{
    /*The computer host sends the command to set the encoder Offset , 
    that the encoder Offset to be written is the type of uint16_t, and value range of the 14bit encoder is 0~16383.*/
    offset &= encoder_mask_;
    this -> write_temp_enc_offset_cmd_[5] = ((offset          ) & 0xFF);     // *(uint8_t *)(&encoderOffset)
    this -> write_temp_enc_offset_cmd_[6] = ((offset >> 8     ) & 0xFF);     // *((uint8_t *)(&encoderOffset)+1)
    
    this -> client_.calculateCheckSum(this -> write_temp_enc_offset_cmd_, 5, 6, this -> write_temp_enc_offset_cmd_[7]);

    this -> client_.send(this -> write_temp_enc_offset_cmd_);
    usleep(1000000);
    this -> getPosition();
}


auto Motor::setPermanentPositionZero() -> void{
    /*Writes the current encoder position of the motor into ROM as the initial position.
    Attention:
    1. This command needs to restart to take effect.
    2. This command will write zero point into ROM of the driver, multiple writing will affect the chip life, which is not recommended for frequent use
    */

    this -> client_.send(this -> write_perm_enc_offset_cmd_);
    usleep(1000000);
    this -> getPosition();
}

