cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-O2 -Wall -Wextra")

project(${PARENT_PROJECT_NAME}_gyroscooter_node)

set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)


find_package(PkgConfig)
# pkg_check_modules(SERIAL libserial)
find_package(CppLinuxSerial REQUIRED)

include_directories(${LibSerial_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

add_executable(
    ${PROJECT_NAME} 
    main.cxx 
    PortHandler.hxx 
    PortHandler.cxx 

    Motor.hxx
    # Motor.cxx

    Base.hxx 
    Base.cxx 
    Base_Model.cxx
    Base_PID.cxx
    Base_Acceleration.cxx
    Base_Position.cxx
    Base_State.cxx
    Base_Operation.cxx
    Base_Torque_Control.cxx
    Base_Speed_Control.cxx
    Base_Position_Control.cxx


    )


target_include_directories(${PROJECT_NAME} PRIVATE ${SERIAL_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME}  PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME}  CppLinuxSerial::CppLinuxSerial ${CMAKE_THREAD_LIBS_INIT})