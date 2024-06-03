#include "PortHandler.hxx"

using namespace mn::CppLinuxSerial;

PortHandler::PortHandler(std::string port_address){
    this -> setPort(port_address);
}

auto PortHandler::setPort(std::string port_address) -> void{
    this -> serial_port_ = SerialPort(port_address, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    this -> serial_port_.SetTimeout(10); // Block for up to 10ms to receive data
	this -> serial_port_.Open();
}

PortHandler::~PortHandler(){
    serial_port_.Close();
}

auto PortHandler::send(const std::vector<uint8_t>& command) -> void{
    this -> serial_port_.WriteBinary(command);
    usleep(5000); // 5ms delay between 2 consecutive "send/receive" calls.
}

auto PortHandler::receive(std::vector<uint8_t>& response) -> void{
    usleep(5000); // 5ms delay between 2 consecutive "send/receive" calls.
    this -> serial_port_.ReadBinary(response);
}


auto PortHandler::calculateCheckSum(const std::vector<uint8_t>& command, uint8_t&& start_index, uint8_t&& end_index, uint8_t& res) -> void{
    this -> sum_ &= 0;
    for (this -> index_ = start_index; this -> index_ <= end_index; this -> index_++) this -> sum_ += command[this -> index_];
    res = ((this -> sum_ & 0xFF)
        // + (this -> sum_ >> 8)
        );
}




