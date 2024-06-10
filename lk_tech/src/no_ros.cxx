#include "Motor.hxx"
#include "SignalHandler.hxx"

#include <iostream>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <exception>


struct StudentController{
    StudentController(std::string port_address, std::uint8_t device_id): motor_{Motor(port_address, device_id)}{
        setHardwarePID();
        setHardwareAcc(0);
    }

    auto setTorque(const float& value) -> void{
        motor_.torqueControl(value);
        temperature_ = motor_.temperature;
        torque_ = motor_.torque;
        speed_ = motor_.speed;
        position_ = motor_.position;
    }

    
    /***************************************/
    // The main function(torqueControl). Everything happens here...!
    /***************************************/
    auto torqueControl() -> void{
            float value{1};
            setTorque(value);
    }

    auto stop() -> void{
        motor_.stop();
    }

    private:
        auto setHardwarePID() -> void{
            // Values Should be between 0 and 255 
            // Don't use this member function in any loop as it writes to the motor's buffer and is time consuming.
            pid_.position_Kp    = 100;
            pid_.position_Ki    = 100;
            pid_.speed_Kp       = 50;
            pid_.speed_Ki       = 70;
            pid_.torque_Kp      = 50;
            pid_.torque_Ki      = 50;
            motor_.setPID(pid_);

            // Store the pid values returned from the motor!
            std::memcpy(&pid_, &motor_.pid, sizeof(pid_));
        }

        auto setHardwareAcc(const std::int32_t& value) -> void{
            // Don't use this member function in any loop as it writes to the motor's buffer and is time consuming.
            motor_.setAcc(value);
            acceleration_ = motor_.acceleration;
        }

    private:
        Motor motor_;
        PID pid_;
        float temperature_{};
        float torque_{};
        float speed_{};
        float position_{};
        float acceleration_{};
};


auto main() -> int{
    
    SignalHandler handler;
    StudentController sc("/dev/ttyUSB0", 0x03);
    handler.run(std::bind(&StudentController::torqueControl, &sc));
    sc.stop(); 

    return 0;
}