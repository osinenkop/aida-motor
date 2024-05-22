#include "Motor.hxx"


Motor::~Motor(){}

Motor::Motor(std::string port_address, uint8_t device_id): device_id_{device_id}, client_{PortHandler(port_address)}{

    this -> response_str.resize(255);

    this -> initModelCmd();
    this -> initPIDCmd();
    this -> initAccelerationCmd();
    this -> initPositionCmd();
    

}



auto Motor::removeSpace(std::vector<uint8_t>& arr) -> void{
    // this -> response_str.clear();
    std::copy_if(arr.begin(), arr.end(),
                    // std::back_inserter(this -> response_str),
                    this -> response_str.begin(),
                    [](uint8_t x) { return '\0' != x; });
}






