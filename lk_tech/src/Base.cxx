#include "Base.hxx"


Base::~Base(){
    this -> stop();
    // this -> shutdown();
}

Base::Base(std::string port_address, std::uint8_t device_id): device_id_{device_id}, client_{PortHandler(port_address)}{

    this -> initModelCmd();
    this -> initPIDCmd();
    this -> initAccelerationCmd();
    this -> initPositionCmd();
    this -> initStateCmd();
    this -> initOperationCmd();

    this -> initTorqueControlCmd();
    this -> initSpeedControlCmd();
    this -> initPositionControlCmd();
    
    

    this -> turnOn();
}





