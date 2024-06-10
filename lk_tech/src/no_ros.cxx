#include "Motor.hxx"
#include "SignalHandler.hxx"

#include <iostream>
#include <cstdint>
#include <algorithm>
#include <vector>


struct StudentController{
    StudentController(std::string port_address, std::uint8_t device_id): motor_{Motor(port_address, device_id)}{
        setHardwarePID();
        setHardwareAcc();
    }

    auto setTorque(const float& value) -> void{
        motor_.torqueControl(value);
        temperature_ = motor_.temperature;      // Unit: celcius
        torque_ = motor_.torque;                // Unit: Amp,       Range: -18~18
        speed_ = motor_.speed;                  // Unit: rps,       Range: -5.4~5.4
        position_ = motor_.position;            // Unit: degree,    Range: 0.0~359.99
    }

    
    /***************************************/
    // The main function(torqueControl). Everything happens here...!
    /***************************************/
    auto torqueControl() -> void{
        /*Unit: Amps, Range: -18~18*/
        float value{1};
        setTorque(value);
    }

    auto stop() -> void{
        motor_.stop();
    }

    private:
        auto setHardwarePID() -> void{
            // Values Should be between 0 and 255 
            // Don't use this member function in any loop as it writes to the motor's memory and is time consuming.
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

        auto setHardwareAcc() -> void{
            /*Don't use this member function in any loop as it writes to the motor's memory and is time consuming.*/ 
            /*Unit: 1dps/s */
            motor_.setAcc(100);
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