#include "Base.hxx"

auto Base::initStateCmd() -> void{
    this -> data_.read_temperature_and_voltage_cmd_ = {0x3E, 0x9A, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_temperature_and_voltage_cmd_, 0, 3, this -> data_.read_temperature_and_voltage_cmd_[4]);
    this -> data_.read_temperature_and_voltage_response_.resize(13, 0x00);

    this -> data_.clear_error_cmd_ = {0x3E, 0x9B, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.clear_error_cmd_, 0, 3, this -> data_.clear_error_cmd_[4]);
    this -> data_.clear_error_response_.resize(13, 0x00);

    this -> data_.read_temperature_and_torque_speed_pose_cmd_ = {0x3E, 0x9C, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_temperature_and_torque_speed_pose_cmd_, 0, 3, this -> data_.read_temperature_and_torque_speed_pose_cmd_[4]);
    this -> data_.read_temperature_and_torque_speed_pose_response_.resize(13, 0x00);

    this -> data_.read_temperature_and_phase_current_cmd_ = {0x3E, 0x9D, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_temperature_and_phase_current_cmd_, 0, 3, this -> data_.read_temperature_and_phase_current_cmd_[4]);
    this -> data_.read_temperature_and_phase_current_response_.resize(13, 0x00);
}


auto Base::getTemperatureVoltage() -> void{
    /*This command reads the current motor's temperature, voltage, and error status flags.*/

    this -> client_.send(this -> data_.read_temperature_and_voltage_cmd_);
    this -> client_.receive(this -> data_.read_temperature_and_voltage_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following parameters:
    1. Motor temperature (int8_t type, unit 1°C/LSB).
    2. Voltage (uint16_t, unit 0.1v /LSB).
    3. ErrorState (uint8_t type, each bit represents different motor state)*/
    
    this -> temperature     =   this -> data_.read_temperature_and_voltage_response_[5];

    this -> voltage         =   (
                                (static_cast<uint16_t>(this -> data_.read_temperature_and_voltage_response_[7])      ) |
                                (static_cast<uint16_t>(this -> data_.read_temperature_and_voltage_response_[8]) << 8 )
                                );

    /*
    error_state[0] == 0  -> Normal
    error_state[0] == 1  -> Low voltage

    error_state[3] == 0  -> Normal
    error_state[3] == 1  -> Over temperature
    */

    this -> error_state     =   this -> data_.read_temperature_and_voltage_response_[11];
}


auto Base::cleanError() -> void{
    /*This command clears the current motor error state and the motor returns when it is received.*/

    this -> client_.send(this -> data_.clear_error_cmd_);
    this -> client_.receive(this -> data_.clear_error_response_);
    
    /*The motor replies to the host after receiving the command, and the frame data contains the following parameters.
    1. Motor temperature (int8_t type, unit 1°C/LSB).
    2. Voltage (uint16_t, unit 0.1v /LSB).
    3. ErrorState (uint8_t type, each bit represents different motor state)*/
    
    this -> temperature     =   this -> data_.clear_error_response_[5];

    this -> voltage         =   (
                                (static_cast<uint16_t>(this -> data_.clear_error_response_[7])      ) |
                                (static_cast<uint16_t>(this -> data_.clear_error_response_[8]) << 8 )
                                );

    /*
    error_state[0] == 0  -> Normal
    error_state[0] == 1  -> Low voltage

    error_state[3] == 0  -> Normal
    error_state[3] == 1  -> Over temperature
    */

    this -> error_state     =   this -> data_.clear_error_response_[11];
}



auto Base::getTorqueSpeedPose() -> void{
    /*This command reads the current motor temperature, torque, speed, encoder position.*/

    this -> client_.send(this -> data_.read_temperature_and_torque_speed_pose_cmd_);
    this -> client_.receive(this -> data_.read_temperature_and_torque_speed_pose_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following parameters.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Torque current IQ of the motor (int16_t type, range -2048~2048, corresponding to the actual
       torque current range -33A ~33A).
    3. Motor speed (int16_t type, 1dps/LSB).
    4. Encoder position value (uint16_t type, the value range of 14bit encoder is 0~16383).*/

    this -> temperature = this -> data_.read_temperature_and_torque_speed_pose_response_[5];

    this -> torque    = (
                        (static_cast<int16_t>(this -> data_.read_temperature_and_torque_speed_pose_response_[6])            ) |
                        (static_cast<int16_t>(this -> data_.read_temperature_and_torque_speed_pose_response_[7]) << 8       ) 
                        );


    this -> speed     = (
                        (static_cast<int16_t>(this -> data_.read_temperature_and_torque_speed_pose_response_[8])            ) |
                        (static_cast<int16_t>(this -> data_.read_temperature_and_torque_speed_pose_response_[9]) << 8       ) 
                        );


    this -> position  = (
                        (static_cast<uint16_t>(this -> data_.read_temperature_and_torque_speed_pose_response_[10])           ) |
                        (static_cast<uint16_t>(this -> data_.read_temperature_and_torque_speed_pose_response_[11]) << 8      ) 
                        ) & this -> data_.encoder_mask_;
}


auto Base::getPhaseCurrent() -> void{
    /*This command reads the current motor temperature and phase current data.*/

    this -> client_.send(this -> data_.read_temperature_and_phase_current_cmd_);
    this -> client_.receive(this -> data_.read_temperature_and_phase_current_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following parameters.
    1. Motor temperature (int8_t type, 1°C/LSB).
    2. Phase A current data, data type int16_t, corresponding to the actual phase current 1A/64LSB.
    3. Phase B current data, data type int16_t, corresponding to the actual phase current 1A/64LSB.
    4. Phase C current data, data type int16_t, corresponding to the actual phase current 1A/64LSB.*/

    this -> temperature     =   this -> data_.read_temperature_and_phase_current_response_[5];

    this -> phase_a_current =   (
                                (static_cast<int16_t>(this -> data_.read_temperature_and_phase_current_response_[6])      )     |
                                (static_cast<int16_t>(this -> data_.read_temperature_and_phase_current_response_[7]) << 8  )
                                );

    this -> phase_b_current =   (
                                (static_cast<int16_t>(this -> data_.read_temperature_and_phase_current_response_[8])       )     |
                                (static_cast<int16_t>(this -> data_.read_temperature_and_phase_current_response_[9]) << 8  )
                                );

    this -> phase_c_current =   (
                                (static_cast<int16_t>(this -> data_.read_temperature_and_phase_current_response_[10])       )    |
                                (static_cast<int16_t>(this -> data_.read_temperature_and_phase_current_response_[11]) << 8  )
                                );
}
