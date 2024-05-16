#ifndef __PORT_HANDLER_HXX__
#define __PORT_HANDLER_HXX__

#include <CppLinuxSerial/SerialPort.hpp>
using namespace mn::CppLinuxSerial;

#include <vector>
#include <string>
#include <unistd.h>

class PortHandler{
    public:
        PortHandler(std::string);
        ~PortHandler();
        auto setPort(std::string) -> void;
        auto send(const std::vector<uint8_t>&) -> void;
        auto receive(std::vector<uint8_t>&) -> void;
        // auto cleanBuffer() -> void;

        auto calculateCheckSum(const std::vector<uint8_t>&, uint8_t&&, uint8_t&&, uint8_t& ) -> void;

    private:
        SerialPort serial_port_;
        uint16_t sum_;
        uint8_t index_;
        std::string port_address_;
        uint8_t available_bytes_;
};

#endif