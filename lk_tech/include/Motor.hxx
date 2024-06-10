#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include <string>
#include <cstdint>
#include <cassert>
#include "Base.hxx"
#include "Data.hxx"


class Motor{
    public:
        Motor(std::string, std::uint8_t);
        ~Motor();
        auto setPID(const PID&) -> void;
        auto getPID() -> const PID&;

        auto setAcc(const std::int32_t&) -> void;
        auto getAcc() -> std::int32_t&;

        auto setPositionOffset(const float&) -> void;
        auto getPosition() -> float;
        auto getState() -> void;
        auto getCurrent() -> void;

        auto torqueControl(const float&) -> void;
        auto speedControl(const float&) -> void;
        auto positionControl(const float&, const float&) -> void;

        auto getProductInfo() -> const std::string&;
        auto stop() -> void;
        

    private:
        auto convertCurrent(const std::int16_t&) -> float;

        template <typename In, typename Out>
        auto convertTorque(const In&, bool&&) -> Out;

        template <typename In, typename Out>
        auto convertSpeed(const In&, bool&&) -> Out;

        template <typename In, typename Out>
        auto convertPosition(const In&, bool&&) -> Out;

        auto collectTorqueSpeedPoseData() -> void;
        auto sgn(const float&) -> bool;
        
    public:
        Base base;
        PID pid;
        std::string model_name;

        float temperature{};
        float torque{};
        float speed{};
        float acceleration{};
        float position{};

        std::uint8_t error_state{};
        float voltage{};

        float phase_a_current{}, phase_b_current{}, phase_c_current{};

        std::string port_address;
        uint8_t device_id;

    private:
    bool model_name_exist{};
    const float LSB{5.2 / (1 << 14)};
    const std::uint8_t reduction_ratio{8};
    const std::uint16_t fourteen_bit_resolution{(1 << 14)};

    const float current_limit{18};

};


#endif