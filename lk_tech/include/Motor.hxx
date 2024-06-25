#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include <string>
#include <cstdint>
#include <cassert>
#include <cmath>
#include "Base.hxx"
#include "Data.hxx"

#include <numbers> // pi

class Motor{
    public:
        Motor(std::string, std::uint8_t, bool debug = true);
        ~Motor();
        auto exists() -> bool;

        auto setPID(const MOTOR_PID&) -> void;
        auto getPID() -> const MOTOR_PID&;

        auto setAcc(const std::int32_t&) -> void;
        auto setAcc(const std::int32_t&&) -> void;
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
        auto shutdown() -> void;
        

    private:
        auto convertCurrent(const std::int16_t&) -> float;

        template <typename In, typename Out>
        auto convertTorque(const In&, bool&&) -> Out;

        template <typename In, typename Out>
        auto convertSpeed(const In&, bool&&) -> Out;

        template <typename In, typename Out>
        auto convertPosition(const In&, bool&&) -> Out;

        auto collectTorqueSpeedPoseData() -> void;
        auto clockWise(const float&) -> bool;
        
    public:
        Base base;
        MOTOR_PID pid;
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
        bool status;

    private:
        bool model_name_exist{};
        const float LSB{5.2 / (1 << 14)};
        const std::uint8_t reduction_ratio{8};
        const std::uint16_t fourteen_bit_resolution{(1 << 14)};
        const std::uint16_t thirteen_bit_resolution{(1 << 13)};

        std::int32_t shifted_value{}, shifted_prev_value{};

        const float torque_limit{18}; // Amps
        const float speed_limit{5.4}; // Rounds/Sec

        std::int16_t encoder_counter{0};
        std::uint16_t old_encoder_value{};
        bool debug_;

};


#endif