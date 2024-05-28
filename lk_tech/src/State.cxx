#include "Motor.hxx"

auto Motor::initStateCmd() -> void{
    this -> data_.read_temperature_and_voltage_cmd_ = {0x3E, 0x9A, this->device_id_, 0x00, 0x00};
    this -> client_.calculateCheckSum(this -> data_.read_temperature_and_voltage_cmd_, 0, 3, this -> data_.read_temperature_and_voltage_cmd_[4]);
    this -> data_.read_temperature_and_voltage_response_.resize(255, 0x00);
}


auto Motor::readTemperatureAndVoltage() -> void{
    /*This command reads the current motor's temperature, voltage, and error status flags.*/

    this -> client_.send(this -> data_.read_temperature_and_voltage_cmd_);
    this -> client_.receive(this -> data_.read_temperature_and_voltage_response_);
    this -> removeSpace(this -> data_.read_temperature_and_voltage_response_);

    /*The motor replies to the host after receiving the command, and the frame data contains the following parameters:
    1. Motor temperature (int8_t type, unit 1°C/LSB).
    2. Voltage (uint16_t, unit 0.1v /LSB).
    3. ErrorState (uint8_t type, each bit represents different motor state)*/
    
    this -> temperature     =   this -> response_str[5] * LSB;

    this -> voltage         =   (
                                (this -> response_str[7]      ) +
                                (this -> response_str[8] << 8 )
                                ) * (0.1*LSB);

    /*
    error_state[0] == 0  -> Normal
    error_state[0] == 1  -> Low voltage

    error_state[3] == 0  -> Normal
    error_state[3] == 1  -> Over temperature
    */

    this -> error_state     =   this -> response_str[11];
}


