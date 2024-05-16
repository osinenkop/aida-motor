
#include "Motor.hxx"
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

    Motor m("/dev/ttyUSB0", 0x03);

    for(int i{}; i < 1; i++){
        // std::array<uint8_t, 6> arr{100, 100, 50, 70, 50, 51};
        // m.setTemporaryPID({100, 100, 50, 70, 50, 13});

        // m.setPermanentPID({100, 100, 50, 70, 50, 50});

        // m.getPID();
        // printf("%d, %d, %d, %d, %d, %d\n", m.pid_value[0], m.pid_value[1], m.pid_value[2] ,m.pid_value[3], m.pid_value[4], m.pid_value[5]); 
        
        m.getAcceleration();
        printf("%d\n", m.acceleration);
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


