#ifndef __Base_HXX__
#define __Base_HXX__

#include "PortHandler.hxx"
#include "Data.hxx"
#include <memory>
#include <cstdint>

// Test Checks
#include <cassert>

struct productInfo
{
std::uint8_t driver_name [20]; // Driver name
std::uint8_t motor_name [20]; // Name of motor
std::uint8_t hardware_version; // drive the hardware version
std::uint8_t firmware_version; // firmware version
};

class Base{
    public:
        Base(std::string, std::uint8_t);
        ~Base();

        auto initModelCmd() -> void;
        auto getModel() -> void;

        auto initPIDCmd() -> void;
        auto getPID() -> void;
        auto setTemporaryPID(const std::array<std::uint8_t, 6>&) -> void;
        auto setPermanentPID(const std::array<std::uint8_t, 6>&) -> void;

        auto initAccelerationCmd() -> void;
        auto getAcceleration() -> void;
        auto setAcceleration(std::uint32_t&) -> void;

        auto initPositionCmd() -> void;
        auto getPosition() -> void;
        auto setTemporaryPositionZero(std::uint16_t&) -> void;
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

        auto initTorqueControlCmd() -> void;
        auto openLoopTorqueControl(const std::int16_t&) -> void;  // Not Working...
        auto closedLoopTorqueControl(const std::int16_t&) -> void;  // Not Working...

        auto initSpeedControlCmd() -> void;
        auto closedLoopSpeedControl(const std::int32_t&) -> void;

        auto initPositionControlCmd() -> void;
        auto closedLoopMultiPositionControl(const std::int64_t&) -> void;
        auto closedLoopMultiPositionControl(const std::int64_t&, const std::uint32_t&) -> void;
        auto closedLoopSinglePositionControl(const std::uint16_t&, const std::int8_t&) -> void;
        auto closedLoopSinglePositionControl(const std::uint16_t&,  const std::int8_t&, const std::uint32_t&) -> void;
        // auto closedLoopIncrementalPositionControl(const std::int32_t&) -> void;  /*NOT WORKING*/
        auto closedLoopIncrementalPositionControl(const std::int32_t&, const std::uint32_t&) -> void;

    private:
        PortHandler client_;
        Data data_;

    public:
        std::uint8_t device_id_;
        productInfo product_info;
        

    public:
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