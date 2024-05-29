#include "Base.hxx"


Base::~Base(){
    // this -> stop();
    // this -> shutdown();
}

Base::Base(std::string port_address, uint8_t device_id): device_id_{device_id}, client_{PortHandler(port_address)}{

    this -> response_str.resize(255);

    this -> initModelCmd();
    this -> initPIDCmd();
    this -> initAccelerationCmd();
    this -> initPositionCmd();
    this -> initStateCmd();
    this -> initOperationCmd();
    this -> initTorqueCmd();

    this -> turnOn();
}


auto Base::removeSpace(const std::vector<uint8_t>& arr) -> void{
    this -> response_str.clear();
    std::copy_if(arr.begin(), arr.end(),
                    // std::back_inserter(this -> response_str),
                    this -> response_str.begin(),
                    [](uint8_t x) { return '\0' != x; });
}






