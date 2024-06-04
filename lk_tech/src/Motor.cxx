#include "Motor.hxx"
#include <cstring>

Motor::Motor(std::string port_address, std::uint8_t device_id): base{Base(port_address, device_id)}{}

Motor::~Motor(){
}

template <typename T>
void removeSpace(std::string& a, const T b, int&& n, int&& j){
    if(std::is_same_v<T, std::uint8_t>) {a[j] = b;}
    else for (int i{}; i < n; i++){a[j++] = b[i];}
}

auto Motor::getProductInfo() -> const std::string&{
    this -> base.getModel();

    this -> model_name.clear();
    this -> model_name.resize(43);

    for (int i{}, j{}; i < 20; i++, j++){  this -> model_name[j] = this -> base.product_info.driver_name[i];}
    for (int i{}, j{21}; i < 20; i++, j++){this -> model_name[j] = this -> base.product_info.motor_name[i];}
    this -> model_name[41] = this -> base.product_info.hardware_version;
    this -> model_name[42] = this -> base.product_info.firmware_version;

    this -> model_name.erase(std::remove_if(this -> model_name.begin(), this -> model_name.end(),
                             [](unsigned char x) { return x == '\0'; }),this -> model_name.end());
    return this -> model_name;
}



