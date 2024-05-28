#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include "PortHandler.hxx"
#include "Data.hxx"
#include <memory>
#include <cstdint>

class Motor{
    public:
        Motor(std::string, std::uint8_t);
        ~Motor();
        // void Send(const int16_t, const int16_t);
        std::string Receive();

        auto initModelCmd() -> void;
        auto getModel() -> void;

        auto initPIDCmd() -> void;
        auto getPID() -> void;
        auto setTemporaryPID(const std::array<std::uint8_t, 6>&) -> void;
        auto setTemporaryPID(const std::array<std::uint8_t, 6>&&) -> void;
        auto setPermanentPID(const std::array<std::uint8_t, 6>&) -> void;
        auto setPermanentPID(const std::array<std::uint8_t, 6>&&) -> void;

        auto initAccelerationCmd() -> void;
        auto getAcceleration() -> void;
        auto setAcceleration(std::uint32_t&) -> void;
        auto setAcceleration(std::uint32_t&&) -> void;

        auto initPositionCmd() -> void;
        auto getPosition() -> void;
        auto setTemporaryPositionZero(std::uint16_t&) -> void;
        auto setTemporaryPositionZero(std::uint16_t&&) -> void;
        auto setPermanentPositionZero() -> void;


        auto initStateCmd() -> void;
        auto getMotorMultiTurnAngle() -> void;
        auto getMotorSingleTurnAngle() -> void;

        auto readTemperatureAndVoltage() -> void;

        // Utility
        auto removeSpace(const std::vector<std::uint8_t>&) -> void;


    
    private:
        std::uint8_t device_id_;
        PortHandler client_;
        Data data_;
        std::string response_str;
        std::uint8_t reduction_ratio{8};
        double LSB{(60. - 12.) / (1 << 14)};
        

    public:
        std::string model;
        std::array<std::uint8_t, 6> pid_value;
        int32_t acceleration{};
        std::uint16_t raw_position{}, position{}, position_offset{};
        std::uint64_t multi_turn_angle{};
        std::uint64_t single_turn_angle{};
        std::int8_t temperature;
        std::uint16_t voltage;
        std::uint8_t error_state;



};

#endif