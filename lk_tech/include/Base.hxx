#ifndef __Base_HXX__
#define __Base_HXX__

#include "PortHandler.hxx"
#include "Data.hxx"
#include <memory>
#include <cstdint>

class Base{
    public:
        Base(std::string, std::uint8_t);
        ~Base();
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
        auto getMultiTurnAngle() -> void;
        auto getSingleTurnAngle() -> void;

        auto initStateCmd() -> void;
        auto getTemperatureVoltage() -> void;
        auto cleanError() -> void;
        auto getTorqueSpeedPose() -> void;
        auto getPhaseCurrent() -> void;

        auto initOperationCmd() -> void;
        auto shutdown() -> void;
        auto stop() -> void;
        auto turnOn() -> void;

        auto initTorqueCmd() -> void;
        auto openLoopTorque(const std::int16_t&) -> void;  // Not Working...
        auto openLoopTorque(const std::int16_t&&) -> void; // Not Working...

        auto closedLoopTorque(const std::int16_t&) -> void;  // Not Working...
        auto closedLoopTorque(const std::int16_t&&) -> void; // Not Working...


        // Utility
        auto removeSpace(const std::vector<std::uint8_t>&) -> void;


    
    private:
        std::uint8_t device_id_;
        PortHandler client_;
        Data data_;
        // std::string response_str;
        std::uint8_t reduction_ratio{8};
        

    public:
        std::string response_str;

        std::string model;
        std::array<std::uint8_t, 6> pid_value;
        int32_t acceleration{};
        std::uint16_t raw_position{}, position{}, position_offset{};
        std::int64_t multi_turn_angle{};
        std::uint64_t single_turn_angle{};
        std::int8_t temperature{};
        std::uint16_t voltage{};
        std::uint8_t error_state{};
        std::int16_t torque{};
        std::int16_t speed{};

        std::int16_t phase_a_current{}, phase_b_current{}, phase_c_current{};
        bool shutdown_success{}, stop_success{}, turn_on_success{};

        std::int16_t power{};
};

#endif