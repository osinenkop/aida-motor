
#include "Base.hxx"
#include <iostream>
#include <algorithm>
#include <vector>
#include <unistd.h>

#include <iterator>
// template<int N>
template<typename T>
std::ostream& operator<< (std::ostream& s, const std::vector<T>& arr){
    std::ranges::copy(arr, std::ostream_iterator<T>(s, " "));
    s << "\n";
    return s;
}


int main(){

    // std::vector <int> abc{1,2,3};
    // std::cout << abc;

    Base m("/dev/ttyUSB0", 0x03);

    for(int i{}; i < 1; i++){
        // std::array<uint8_t, 6> arr{100, 100, 50, 70, 50, 52};
        // m.setTemporaryPID(arr);
        // m.setTemporaryPID({100, 100, 50, 70, 50, 13});

        // m.setPermanentPID({100, 100, 50, 70, 50, 50});

        // m.getPID();
        // printf("%d, %d, %d, %d, %d, %d\n", m.pid_value[0], m.pid_value[1], m.pid_value[2] ,m.pid_value[3], m.pid_value[4], m.pid_value[5]); 
        
//         m.getAcceleration();
// printf("%d\n", m.acceleration);

        // m.getAcceleration();
        // printf("%d\n", m.acceleration);
        // uint16_t offs{0};
        // m.setTemporaryPositionZero(10);

        // m.operate();
        // printf("%d\n", m.operate_success);

        // m.stop();
        // printf("%d\n", m.stop_success);

        // m.shutdown();
        // printf("%d\n", m.shutdown_success);

        // m.cleanError();
        // m.readTemperatureVoltage();
        // printf("Temp: %d, Voltage : %d\n", m.temperature, m.voltage);
        // m.getSingleTurnAngle();
        // m.getPosition();
        // m.readTorqueSpeedPose();
        // std::int16_t a{200};
        // m.openLoopTorque(a);
        m.turnOn();
        printf("%d\n", m.turn_on_success);
        // printf("len: %d\n", m.response_str[3]);
        // printf("res: %d\n", m.response_str[5]);
        m.closedLoopTorque(100);
        for(int k{}; k < 100; k++){
            m.getTorqueSpeedPose();
            printf("Temp: %d, Torq : %d, Speed: %d, Pose: %d\n", m.temperature, m.torque, m.speed, m.position);
        usleep(10000);}

        m.stop();
            
        // std::clog << "Temp: " << m.temperature << ", Torque :" << m.torque << ", Speed: " << m.speed << ", Pose: " << m.position << "\n";
        // m.getPosition();
        // printf("%d, %d, %d\n", m.position, m.raw_position, m.position_offset);
        
            // m.getSingleTurnAngle();
            // printf("%ld\n", m.single_turn_angle);

            // m.getMotorMultiTurnAngle();
            // printf("%Lf\n", m.multi_turn_angle);
//         m.getModel();
// printf("%s\n", m.model.c_str());
    // m.readTemperatureAndVoltage();
    // printf("%d %d\n", m.temperature, m.voltage);

    // printf("%f\n", m.LSB);
    }
        

    return 0;
}





//// Commands
//// Model
// m.getModel();
// printf("%s\n", m.model.c_str()); 

//// get pid values
// m.getPID();
// printf("%d, %d, %d, %d, %d, %d\n", m.pid_value[0], m.pid_value[1], m.pid_value[2] ,m.pid_value[3], m.pid_value[4], m.pid_value[5]); 

//// set temporary pid values
// std::array<uint8_t, 6> arr{100, 100, 50, 70, 50, 50};
// m.setTemporaryPID(arr);
// // or
// m.setTemporaryPID({100, 100, 50, 70, 50, 50});

//// set permanent pid values
// std::array<uint8_t, 6> arr{100, 100, 50, 70, 50, 50};
// m.setPermanentPID(arr);
// // or
// m.setPermanentPID({100, 100, 50, 70, 50, 50});


//// get acceleration value
// m.getAcceleration()
// printf("%d\n", m.acceleration);


//// set acceleration value
// uint32_t acc{120};
// m.setAcceleration(acc)
// // or
// m.setAcceleration(120)


