#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include "PortHandler.hxx"
#include <memory>
#include <bitset>

class Motor{
    public:
        Motor(std::string, uint8_t);
        ~Motor();
        // void Send(const int16_t, const int16_t);
        std::string Receive();

        auto initModelCmd() -> void;
        auto getModel() -> void;

        auto initPIDCmd() -> void;
        auto getPID() -> void;
        auto setTemporaryPID(const std::array<uint8_t, 6>&) -> void;
        auto setTemporaryPID(const std::array<uint8_t, 6>&&) -> void;
        auto setPermanentPID(const std::array<uint8_t, 6>&) -> void;
        auto setPermanentPID(const std::array<uint8_t, 6>&&) -> void;

        auto initAccelerationCmd() -> void;
        auto getAcceleration() -> void;
        auto setAcceleration(uint32_t&) -> void;
        auto setAcceleration(uint32_t&&) -> void;

        auto initPositionCmd() -> void;
        auto getPosition() -> void;
        auto setTemporaryPositionZero(uint16_t&) -> void;
        auto setTemporaryPositionZero(uint16_t&&) -> void;
        auto setPermanentPositionZero() -> void;


        // Utility
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

    std::vector<uint8_t> read_enc_cmd_;
    std::vector<uint8_t> read_enc_response_;

    std::vector<uint8_t> write_temp_enc_offset_cmd_;
    std::vector<uint8_t> write_perm_enc_offset_cmd_;
    uint16_t encoder_mask_{(1 << 14) - 1};


    public:
    std::string model;
    std::array<uint8_t, 6> pid_value;
    int32_t acceleration{};
    uint16_t raw_position{}, position{}, position_offset{};

};

#endif