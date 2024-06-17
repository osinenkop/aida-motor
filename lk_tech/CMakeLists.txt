cmake_minimum_required(VERSION 3.8)
project(lk_tech)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(
    ${PROJECT_NAME}_motor

    src/main.cxx
    
    
    src/SerialPort.cxx
    src/PortHandler.cxx 
    
    src/Base.cxx 
    src/Base_Model.cxx
    src/Base_PID.cxx
    src/Base_Acceleration.cxx
    src/Base_Position.cxx
    src/Base_State.cxx
    src/Base_Operation.cxx
    src/Base_Torque_Control.cxx
    src/Base_Speed_Control.cxx
    src/Base_Position_Control.cxx
    src/Motor.cxx

    src/MotorManager.cxx

)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_target_dependencies(
  ${PROJECT_NAME}_motor
  rclcpp
  std_msgs
  sensor_msgs
)


target_link_libraries(
  ${PROJECT_NAME}_motor
  ${DEPENDENCIES}
  rcl::rcl
  rclcpp::rclcpp

  pthread
  tbb
)

install(TARGETS
${PROJECT_NAME}_motor
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
