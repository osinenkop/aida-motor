#ifndef __MOTOR_HXX__
#define __MOTOR_HXX__

#include <string>
#include <cstdint>
#include "Base.hxx"


struct PID{
float position_Kp, position_Ki;
float speed_Kp, speed_Ki;
float torque_Kp, torque_Ki;
};

class Motor{
    public:
        Motor(std::string, std::uint8_t);
        ~Motor();
        auto setPID() -> void;
        auto getPID() -> void;

        auto setAcc() -> void;
        auto getAcc() -> void;

        auto getState() -> void;

        auto SpeedControl() -> void;
        auto TorqueControl() -> void;
        auto PositionControl() -> void;

        auto getProductInfo() -> const std::string&;



        Base base;
        std::string model_name;

        float temperature{};
        float torque{};
        float speed{};
        float acceleration{};
        float position{};
        PID pid{};

        std::uint8_t error_state{};
        float voltage{};

        std::string port_address;
        uint8_t device_id;

};


#endif