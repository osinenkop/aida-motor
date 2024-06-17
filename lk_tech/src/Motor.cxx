#include "Motor.hxx"

#include <cstring>
#include <algorithm>

Motor::Motor(std::string port_address, std::uint8_t device_id, bool debug): base{Base(port_address, device_id)}, debug_{debug}{
    if(this -> exists() and (not this -> debug_)){this -> base.turnOn();}
}

Motor::~Motor(){
    if (not this -> debug_){
        this -> stop();
        this -> shutdown();
    }
}

auto Motor::exists() -> bool{
    this -> model_name_exist = (this -> getProductInfo().size() > 0);
    return (this -> model_name_exist)? this -> model_name_exist : this -> getProductInfo().size() > 0;
}

template <typename T>
void removeSpace(std::string& a, const T b, int&& n, int&& j){
    if(std::is_same_v<T, std::uint8_t>) {a[j] = b;}
    else for (int i{}; i < n; i++){a[j++] = b[i];}
}

auto Motor::getProductInfo() -> const std::string&{
    if (! this -> model_name_exist){
        this -> base.getModel();

        this -> model_name.clear();
        this -> model_name.resize(43);

        for (int i{}, j{}; i < 20; i++, j++){  this -> model_name[j] = this -> base.product_info.driver_name[i];}
        for (int i{}, j{21}; i < 20; i++, j++){this -> model_name[j] = this -> base.product_info.motor_name[i];}
        this -> model_name[41] = this -> base.product_info.hardware_version;
        this -> model_name[42] = this -> base.product_info.firmware_version;

        this -> model_name.erase(std::remove_if(this -> model_name.begin(), this -> model_name.end(),
                                [](unsigned char x) { return x == '\0'; }),this -> model_name.end());
        this -> model_name_exist = true;
        }
    
    return this -> model_name;
}


auto Motor::getPID() -> const MOTOR_PID&{
    this -> base.getPID(); // values are between 0-255
    return this -> base.pid_value;
}


auto Motor::setPID(const MOTOR_PID& pid_value) -> void{
        this -> base.setTemporaryPID({  pid_value.position_Kp, pid_value.position_Ki,
                                        pid_value.speed_Kp,    pid_value.speed_Ki,
                                        pid_value.torque_Kp,   pid_value.torque_Ki}); // values are between 0~255}
}

auto Motor::getAcc() -> std::int32_t&{
    /*Do it Once only*/
    this -> base.getAcceleration(); // Unit: 1dps/s
    return this -> base.acceleration;
}

auto Motor::setAcc(const std::int32_t& value) -> void{
    /*Do it Once only as it writes the data to the RAM of the motor*/
    this -> base.setAcceleration(value); // Unit: 1dps/s
}

auto Motor::setAcc(const std::int32_t&& value) -> void{
    /*Do it Once only as it writes the data to the RAM of the motor*/
    this -> base.setAcceleration(value); // Unit: 1dps/s
}

auto Motor::setPositionOffset(const float& offset) -> void{
    // Range: 0~359
    if((0 <= offset) and (offset < 360)) {this -> base.setTemporaryPositionZero(static_cast<std::uint16_t>(offset / 360.0) * (fourteen_bit_resolution - 1));}
}

auto Motor::getPosition() -> float{
    this -> base.getSingleTurnAngle(); // Unit: 0.01 degree
    return this -> base.single_turn_angle * 0.01 / this -> reduction_ratio; // degree
}


auto Motor::getState() -> void{
    this -> base.getTemperatureVoltage();
    this -> temperature = this -> base.temperature; // Unit: centigrade
    this -> voltage = this -> base.voltage * 0.1; // Unit: volt
    this -> error_state = this -> base.error_state; 
}

auto Motor::getCurrent() -> void{
    this -> base.getPhaseCurrent();
    this -> phase_a_current = this -> convertCurrent(this -> base.phase_a_current); // Unit: Amp
    this -> phase_b_current = this -> convertCurrent(this -> base.phase_b_current); // Unit: Amp
    this -> phase_c_current = this -> convertCurrent(this -> base.phase_c_current); // Unit: Amp
}

auto Motor::torqueControl(const float& value) -> void{
    // Unit: Amps
    // The value will be clamped to the range ±torque_limit
    this -> base.closedLoopTorqueControl(this -> convertTorque<float, std::int16_t>(std::clamp<float>(value, -this -> torque_limit, this -> torque_limit), false));
    this -> collectTorqueSpeedPoseData();
}

auto Motor::speedControl(const float& value) -> void{
    // Unit:  Rounds/Sec
    this -> base.closedLoopSpeedControl(this -> convertSpeed<float, std::int32_t>(std::clamp<float>(value, -this -> speed_limit, this -> speed_limit), false));
    this -> collectTorqueSpeedPoseData();
}

auto Motor::stop() -> void{
    this -> base.stop();
}

auto Motor::shutdown() -> void{
    this -> base.shutdown();
}

/*Complex and I do not recommend it*/
// auto Motor::positionControl(const float& value, const float& speed) -> void{
//     // Unit: Degree
//     this -> base.closedLoopSinglePositionControl(this -> convertPosition<float, std::int64_t>(value, false), this -> sgn(value), this -> convertSpeed<float, std::uint32_t>(value, false));
//     this -> collectTorqueSpeedPoseData();
// }


auto Motor::collectTorqueSpeedPoseData() -> void{
        this -> temperature = this -> base.temperature; // Unit: 1 centigrade
        this -> torque      = this -> convertTorque<std::int16_t, float>(this -> base.torque, true); // Unit: Amp, Range: -33~33
        this -> speed       = this -> convertSpeed<std::int16_t, float>(this -> base.speed, true); // Unit: rounds/s, Range: -5.4~5.4
        this -> position    = this -> convertPosition<std::uint16_t, float>(this -> base.position, true); // Unit: degree, Range: 0~359.99
}


auto Motor::convertCurrent(const std::int16_t& value) -> float{
    return static_cast<float>(value) / 64.0;
}

auto Motor::sgn(const float& value) -> bool{
    return value <= 0;
}

template <typename In, typename Out>
auto Motor::convertTorque(const In& value, bool&& from_motor) -> Out{
    if (from_motor) {return static_cast<Out>(33.0 * value / 2048.0);}
    else {return static_cast<Out>(2000 * (value / 32.0));} 
}

template <typename In, typename Out>
auto Motor::convertSpeed(const In& value, bool&& from_motor) -> Out{
    if (from_motor) {return static_cast<Out>(value) / 360.0 / this -> reduction_ratio;}
    else {return static_cast<Out>(value * this -> reduction_ratio * 100.0 * 360.0);} // Unit: Rounds/Sec
}

// Pose
template <typename In, typename Out>
auto Motor::convertPosition(const In& value, bool&& from_motor) -> Out{
    if (from_motor) {return 360 * (static_cast<Out>(value) / (fourteen_bit_resolution)) - 1;}
    else return static_cast<Out>(value * 100.0 * this -> reduction_ratio); 
}

















