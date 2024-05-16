#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include "PortHandler.hxx"
#include <memory>

class Motor{
    public:
        Motor(std::string, uint8_t);
        ~Motor();
        // void Send(const int16_t, const int16_t);
        std::string Receive();

        auto getModel() -> void;
        auto getPID() -> void;
        auto setTemporaryPID(const std::array<uint8_t, 6>&) -> void;
        auto setTemporaryPID(const std::array<uint8_t, 6>&&) -> void;
        auto setPermanentPID(const std::array<uint8_t, 6>&) -> void;
        auto setPermanentPID(const std::array<uint8_t, 6>&&) -> void;
        auto getAcceleration() -> void;
        auto setAcceleration(uint32_t&) -> void;
        auto setAcceleration(uint32_t&&) -> void;
        auto removeSpace(std::vector<uint8_t>&) -> void;


    
    private:
        uint8_t device_id_;
        PortHandler client_;

    private:
    std::string response_str;
    std::vector<uint8_t> model_cmd_;
    std::vector<uint8_t> model_response_;


    std::vector<uint8_t> read_pid_cmd_;
    std::vector<uint8_t> read_pid_response_;

    std::vector<uint8_t> write_pid_ram_cmd_;
    std::vector<uint8_t> write_pid_rom_cmd_;

    std::vector<uint8_t> read_acc_cmd_;
    std::vector<uint8_t> read_acc_response_;

    std::vector<uint8_t> write_acc_cmd_;



    public:
    std::string model;
    std::array<uint8_t, 6> pid_value;
    uint32_t acceleration{};

};

#endif